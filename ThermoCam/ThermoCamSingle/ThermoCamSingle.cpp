#include <opencv/cv.h>
#include <opencv/highgui.h>
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
#include <fstream>
#include <ctime>

PV_INIT_SIGNAL_HANDLER();

bool AcquireImages()
{
	PvResult lResult;	
	PvDeviceInfo *lDeviceInfo = NULL;
	PvSystem lSystem;
	PvStream lStream;
	lSystem.SetDetectionTimeout( 20000 );
	lResult = lSystem.Find();
	if( !lResult.IsOK() )
	{
		cout << "PvSystem::Find Error: " << lResult.GetCodeString().GetAscii();
		return -1;
	}
	PvUInt32 lInterfaceCount = lSystem.GetInterfaceCount();
	for( PvUInt32 x = 0; x < lInterfaceCount; x++ )
	{
		PvInterface * lInterface = lSystem.GetInterface( x );
		cout << "Ethernet Interface " << endl;
		cout << "IP Address: " << lInterface->GetIPAddress().GetAscii() << endl;
		cout << "Subnet Mask: " << lInterface->GetSubnetMask().GetAscii() << endl << endl;
		PvUInt32 lDeviceCount = lInterface->GetDeviceCount();
		for( PvUInt32 y = 0; y < lDeviceCount ; y++ )
		{
			lDeviceInfo = lInterface->GetDeviceInfo( y );
			cout << "ThermoCam " << endl;
			cout << "IP Address: " << lDeviceInfo->GetIPAddress().GetAscii() << endl;
		}
	}
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
    			lResult = lDevice.NegotiatePacketSize( );
    			if ( !lResult.IsOK() )
    			{
				cout << endl;
        			cout << " Failed to negotiate a packet size setting GevSCPSPacketSize to original value";
        			PvSleepMs( 2500 );
    			}
			cout << endl;
    			cout << "3. Open stream......";
			lResult = lStream.Open( lDeviceInfo->GetIPAddress() );
			if ( !lResult.IsOK() )
			{	
				cout << endl;
				cout << "  Failed to open stream";
				return 0;
			}
			lDevice.SetStreamDestination( lStream.GetLocalIPAddress(), lStream.GetLocalPort() );
			PvInt64 lPayloadSize;
			lDevice.GetGenParameters()->GetIntegerValue( "PayloadSize", lPayloadSize );
			PvBuffer * lBuffer = new PvBuffer();
			lBuffer->Alloc( static_cast<PvUInt32>( lPayloadSize ) );
			PvBuffer *lPtr = NULL; 
			PvImage *lImage = NULL;
			cout << endl;
			cout << "5. Grab one image" << endl;
			lStream.QueueBuffer( lBuffer );
			lDevice.GetGenParameters()->SetIntegerValue( "TLParamsLocked", 1 );
			lDevice.GetGenParameters()->ExecuteCommand( "AcquisitionStart" );
			PvResult lStreamResult;
			lResult = lStream.RetrieveBuffer( &lPtr, &lStreamResult, 10000 );
			lDevice.GetGenParameters()->ExecuteCommand( "AcquisitionStop" );
			lDevice.GetGenParameters()->SetIntegerValue( "TLParamsLocked", 0 );
			PvInt64 lWidth = 0, lHeight = 0;
			PvGenParameterArray *lDeviceParams = lDevice.GetGenParameters();	
			lDeviceParams->GetIntegerValue( "Width", lWidth);
			lDeviceParams->GetIntegerValue( "Height", lHeight);			
			cvNamedWindow("OpenCV: ThermoCam",CV_WINDOW_NORMAL);
			cv::Mat raw_lImage(cv::Size(lWidth,lHeight),CV_8U);
			if ( lResult.IsOK() )
			{
				if ( lStreamResult.IsOK() )
				{
					cout << endl;
					cout << "6. Using RGB Filter";		
					lImage=lPtr->GetImage();
					lPtr->GetImage()->Alloc(lImage->GetWidth(),lImage->GetHeight(),PvPixelMono8);
					cout << "  a. Save the original image into ImageOriginal.bmp";			
					PvBufferWriter lBufferWriter;			
					lBufferWriter.Store(lPtr,"ThermoCam.bmp",PvBufferFormatBMP);
				}
				lImage->Attach(raw_lImage.data,lImage->GetWidth(),lImage->GetHeight(),PvPixelMono8);
				//cv::imshow("OpenCV: ThermoCam",raw_lImage);
				cv::FileStorage fs("ThermoCam.xml",cv::FileStorage::WRITE);		
				fs << "raw_lImage" << raw_lImage;
				fs.release();	
				//if(cv::waitKey(1000) >= 0) break;
				lPtr->Free();
			}
		  	lBuffer->Free();
			lDevice.ResetStreamDestination();
			lStream.Close();
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
   	cout << "Connecting to and streaming data from ThermoCam" << endl << endl;
	printf("pid %d\n", (int) getpid());    	
	AcquireImages();
	cout << endl;
    	cout << "<press a key to exit>" << endl;
    	PvWaitForKeyPress();
	return 0;
}


