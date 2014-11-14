13|	# Parameters
14|	# SRC_CPPS: The source CPP files to compile
15|	# EXEC: The executable name
16|	
17|	ifeq ($(SRC_CS) $(SRC_CPPS),)
18|	  $(error No source files specified)
19|	endif
20|	
21|	ifeq ($(EXEC),)
22|	  $(error No executable file specified)
23|	endif
24|	
25|	CXX                 ?= g++
26|	
27|	OPENCV_ROOT	    ?= /usr/local
28|	
29|	PUREGEV_ROOT        ?= ../../..
30|	PV_LIBRARY_PATH      =$(PUREGEV_ROOT)/lib
31|	
32|	CPPFLAGS            += -I$(PUREGEV_ROOT)/include -I$(OPENCV_ROOT)/include
33|	ifdef _DEBUG
34|	    CPPFLAGS  += -g -D_DEBUG
35|	else
36|	    CPPFLAGS  += -O3
37|	endif
38|	CPPFLAGS  += -D_UNIX_ -D_LINUX_
39|	
40|	LDFLAGS             += -L$(PUREGEV_ROOT)/lib -L$(OPENCV_ROOT)/lib        \
41|	                        -lPvBase                     \
42|	                        -lPvDevice                   \
43|	                        -lPvBuffer                   \
44|	                        -lPvGUIUtils                 \
45|	                        -lPvPersistence              \
46|	                        -lPvGenICam                  \
47|	                        -lPvStreamRaw                \
48|	                        -lPvStream                   \
49|	                        -lPvTransmitterRaw           \
50|	                        -lPvVirtualDevice	     \
51|				-lopencv_core		     \
52|				-lopencv_highgui
53|	
54|	
55|	# Conditional linking and usage of the GUI on the sample only when available
56|	ifeq ($(wildcard $(PUREGEV_ROOT)/lib/libPvGUI.so),)
57|	    CPPFLAGS  += -DPV_GUI_NOT_AVAILABLE
58|	else
59|	    LDFLAGS   += -lPvGUI
60|	endif 
61|	
62|	# We want all samples to prevent using deprecated eBUS SDK functions, macros and types
63|	CPPFLAGS  += -DPV_NO_GEV1X_PIXEL_TYPES -DPV_NO_DEPRECATED_PIXEL_TYPES
64|	
65|	# Configure Genicam
66|	GEN_LIB_PATH = $(PUREGEV_ROOT)/lib/genicam/bin/Linux32_i86
67|	LDFLAGS      += -L$(GEN_LIB_PATH)
68|	
69|	
70|	# Configure Qt compilation if any
71|	SRC_MOC              =
72|	MOC			         =
73|	RCC					 =
74|	FILES_QTGUI          = $(shell grep -l QtGui *)
75|	ifneq ($(FILES_QTGUI),)
76|		# This is a sample compiling Qt code
77|	    HAVE_QT=$(shell which qmake-qt4 &>/dev/null ; echo $?)
78|	    ifeq ($(HAVE_QT),1)
79|			# We cannot compile the sample without the Qt SDK!
80|	 		$(error The sample $(EXEC) requires the Qt SDK to be compiled. See share/samples/Readme.txt for more details)
81|	    endif
82|	
83|		# Query qmake to find out the folder required to compile
84|		QT_SDK_BIN        = $(shell qmake-qt4 -query QT_INSTALL_BINS)
85|		QT_SDK_LIB        = $(shell qmake-qt4 -query QT_INSTALL_LIBS)
86|		QT_SDK_INC        = $(shell qmake-qt4 -query QT_INSTALL_HEADERS)
87|		
88|		# We have a full Qt SDK installed, so we can compile the sample
89|		CPPFLAGS         += -I$(QT_SDK_INC)
90|		LDFLAGS          += -L$(QT_SDK_LIB)
91|	
92|	    QT_LIBRARY_PATH   = $(QT_SDK_LIB)
93|	
94|	    FILES_MOC            = $(shell grep -l Q_OBJECT *)
95|	    ifneq ($(FILES_MOC),)
96|		    SRC_MOC           = $(FILES_MOC:%h=moc_%cxx)
97|		    FILES_QRC         = $(shell ls *.qrc)
98|		    SRC_QRC           = $(FILES_QRC:%qrc=qrc_%cxx)
99|	
100|		    OBJS             += $(SRC_MOC:%.cxx=%.o)
101|		    OBJS		     += $(SRC_QRC:%.cxx=%.o)
102|	
103|	        MOC               = $(QT_SDK_BIN)/moc
104|	  	    RCC               = $(QT_SDK_BIN)/rcc
105|	    endif
106|	endif
107|	
108|	# Configure FFmpeg compilation if any
109|	FILES_FFMPEG              = $(shell grep -l avcodec *)
110|	ifneq ($(FILES_FFMPEG),)
111|	    HAVE_FFMPEG = $(shell which ffmpeg &>/dev/null && echo "1")
112|	    ifneq ($(HAVE_FFMPEG),1)
113|	        # We cannot compile the sample without the ffmpeg!
114|	 		$(error The sample $(EXEC) requires ffmpeg to be compiled. See share/samples/Readme.txt for more details)  
115|	    endif
116|	
117|	    # Ensure the proper version of ffmpeg
118|	    FFMPEG_VERSION=$(shell ffmpeg -version)) 
119|	    ifeq (,$(findstring ffmpeg version 0.11.,$(FFMPEG_VERSION)))
120|		    $(error The sample $(EXEC) requires FFmpeg 0.11.x to be compiled. See share/samples/Readme.txt for more details)
121|	    endif
122|	
123|	    # We need to add the library for ffmpeg and we assumed that the PC is configure the find them...
124|	    LDFLAGS          += -lavformat			\
125|	                        -lavcodec 			\
126|	                        -lswscale  
127|	endif
128|	
129|	LD_LIBRARY_PATH       = $(PV_LIBRARY_PATH):$(QT_LIBRARY_PATH):$(GEN_LIB_PATH)
130|	export LD_LIBRARY_PATH
131|	
132|	OBJS      += $(SRC_CPPS:%.cpp=%.o)
133|	
134|	all: $(EXEC)
135|	
136|	clean:
137|		rm -rf $(OBJS) $(EXEC) $(SRC_MOC) $(SRC_QRC)
138|	
139|	moc_%.cxx: %.h
140|		$(MOC) $< -o $@ 
141|	
142|	qrc_%.cxx: %.qrc
143|		$(RCC) $< -o $@
144|	
145|	%.o: %.cxx
146|		$(CXX) -c $(CPPFLAGS) -o $@ $<
147|	
148|	%.o: %.cpp
149|		$(CXX) -c $(CPPFLAGS) -o $@ $<
150|	
151|	$(EXEC): $(OBJS)
152|		$(CXX) $(LDFLAGS) $(OBJS) -o $@
153|	
154|	.PHONY: all clean
