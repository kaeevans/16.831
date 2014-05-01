#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <stdio.h>
#include <PvSampleUtils.h>
#include <PvDevice.h>
#include <PvBuffer.h>
#include <PvStream.h>
#include <PvInterface.h>
#include <PvSystem.h>
#include <PvBufferWriter.h>
#include <PvPixelType.h>
#include <PvString.h>
#include <string>
#include <cstdio>
#include <vector>
#include <iostream>

PV_INIT_SIGNAL_HANDLER();

#define BUFFER_COUNT ( 16 )

bool AcquireImages()
{
    // finding the thermocam
	PvResult lResult;	
	PvDeviceInfo *lDeviceInfo = NULL;

	PvSystem lSystem;

	lSystem.SetDetectionTimeout( 20000 );
	lResult = lSystem.Find();
	if( !lResult.IsOK() )
	{
		cout << "PvSystem::Find Error: " << lResult.GetCodeString().GetAscii();
		return -1;
	}

	PvUInt32 lInterfaceCount = lSystem.GetInterfaceCount();

    // only used when there are multiple thermocams attached
	for( PvUInt32 x = 0; x < lInterfaceCount; x++ )
	{
		PvInterface * lInterface = lSystem.GetInterface( x );

		cout << "Interface " << x << endl;
		cout << "IP Address: " << lInterface->GetIPAddress().GetAscii() << endl;
		cout << "Subnet Mask: " << lInterface->GetSubnetMask().GetAscii() << endl << endl;

		PvUInt32 lDeviceCount = lInterface->GetDeviceCount();

		for( PvUInt32 y = 0; y < lDeviceCount ; y++ )
		{
			lDeviceInfo = lInterface->GetDeviceInfo( y );

			cout << "Device " << y << endl;
			cout << "IP Address: " << lDeviceInfo->GetIPAddress().GetAscii() << endl;
		}
	}

    // connect to thermocam
	if( lDeviceInfo != NULL )
	{
		cout << "Connecting to " << lDeviceInfo->GetIPAddress().GetAscii() << endl;

		PvDevice lDevice;
		lResult = lDevice.Connect( lDeviceInfo );
		if ( !lResult.IsOK() )
		{
			cout << "Unable to connect to " << lDeviceInfo->GetIPAddress().GetAscii() << endl;
		}
		else
		{
			cout << "Successfully connected to " << lDeviceInfo->GetIPAddress().GetAscii() << endl;
    PvGenParameterArray *lDeviceParams = lDevice.GetGenParameters();
    PvGenInteger *lPayloadSize = dynamic_cast<PvGenInteger *>( lDeviceParams->Get( "PayloadSize" ) );
    PvGenCommand *lStart = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStart" ) );
    PvGenCommand *lStop = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStop" ) );
    lDevice.NegotiatePacketSize();
    PvStream lStream;
    cout << "Opening stream to device" << endl;
    lStream.Open( lDeviceInfo->GetIPAddress() );
    PvInt64 lSize = 0;
    lPayloadSize->GetValue( lSize );
    PvUInt32 lBufferCount = ( lStream.GetQueuedBufferMaximum() < BUFFER_COUNT ) ? 
        lStream.GetQueuedBufferMaximum() : 
        BUFFER_COUNT;
    PvBuffer *lBuffers = new PvBuffer[ lBufferCount ];
    for ( PvUInt32 i = 0; i < lBufferCount; i++ )
    {
        lBuffers[ i ].Alloc( static_cast<PvUInt32>( lSize ) );
    }
    lDevice.SetStreamDestination( lStream.GetLocalIPAddress(), lStream.GetLocalPort() );
    PvGenParameterArray *lStreamParams = lStream.GetParameters();
    PvGenInteger *lCount = dynamic_cast<PvGenInteger *>( lStreamParams->Get( "ImagesCount" ) );
    PvGenFloat *lFrameRate = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "AcquisitionRateAverage" ) );
    PvGenFloat *lBandwidth = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "BandwidthAverage" ) );
    for ( PvUInt32 i = 0; i < lBufferCount; i++ )
    {
        lStream.QueueBuffer( lBuffers + i );
    }
    cout << "Resetting timestamp counter..." << endl;
    PvGenCommand *lResetTimestamp = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "GevTimestampControlReset" ) );
    lResetTimestamp->Execute();

    // begin image acquisition
    cout << "Sending StartAcquisition command to device" << endl;
    lResult = lStart->Execute();

    char lDoodle[] = "|\\-|-/";
    int lDoodleIndex = 0;
    PvInt64 lImageCountVal = 0;
    double lFrameRateVal = 0.0;
    double lBandwidthVal = 0.0;

	cout << endl;
    cout << "<press a key to stop streaming>" << endl;

    // image acquisition loop
    while ( !PvKbHit() )
    {
        PvBuffer *lBuffer = NULL;
        PvResult lOperationResult;	
        PvResult lResult = lStream.RetrieveBuffer( &lBuffer, &lOperationResult, 1000 );
        if ( lResult.IsOK() )
        {
        	if(lOperationResult.IsOK())
        	{
				lCount->GetValue( lImageCountVal );
				lFrameRate->GetValue( lFrameRateVal );
				lBandwidth->GetValue( lBandwidthVal );
				PvUInt32 lWidth = 0, lHeight = 0;
				if ( lBuffer->GetPayloadType() == PvPayloadTypeImage )
				{
					PvImage *lImage = lBuffer->GetImage();
					lWidth = lBuffer->GetImage()->GetWidth();
					lHeight = lBuffer->GetImage()->GetHeight();
					cv::Size imsize(lWidth, lHeight); // thermocam image size
                    int sizebytes = 2 * imsize.area() * sizeof(unsigned short);					
                    namedWindow("ThermoCam", WINDOW_AUTOSIZE);				
}

				cout << fixed << setprecision( 1 );
				cout << lDoodle[ lDoodleIndex ];
				cout << " BlockID: " << uppercase << hex << setfill('0') << setw(16) << lBuffer->GetBlockID() << " W: " << dec << lWidth << " H: " 
					 << lHeight << " " << lFrameRateVal << " FPS " << ( lBandwidthVal / 1000000.0 ) << " Mb/s  \r";
        	}
			lStream.QueueBuffer( lBuffer );

        }
        else
        {
            cout << lDoodle[ lDoodleIndex ] << " Timeout\r";
        }

        ++lDoodleIndex %= 6;
    }

    PvGetChar(); 
    cout << endl << endl;
    cout << "Sending AcquisitionStop command to the device" << endl;
    lStop->Execute();
    cout << "Aborting buffers still in stream" << endl;
    lStream.AbortQueuedBuffers();
    while ( lStream.GetQueuedBufferCount() > 0 )
    {
        PvBuffer *lBuffer = NULL;
        PvResult lOperationResult;

        lStream.RetrieveBuffer( &lBuffer, &lOperationResult );

        cout << "  Post-abort retrieved buffer: " << lOperationResult.GetCodeString().GetAscii() << endl;
    }
    cout << "Releasing buffers" << endl;
    delete []lBuffers;
    cout << "Closing stream" << endl;
    lStream.Close();
    cout << "Disconnecting device" << endl;
    lDevice.Disconnect();

    return true;
}
	
	}
	else
	{
		cout << "No device found" << endl;
	}

	return 0;
}

int main()
{
    cout << "Connecting to and Acquiring Images from ThermoCam" << endl << endl;
    AcquireImages();

	cout << endl;
    cout << "<press a key to exit>" << endl;
    PvWaitForKeyPress();

    return 0;
}

