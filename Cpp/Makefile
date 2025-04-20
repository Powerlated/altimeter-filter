BUILD_DIR = build

CXX_INCLUDES = \
-I.

CXX_SOURCES = \
AltimeterFilter/AltimeterFilter.cpp \
UKF/UKF.cpp

CXXFLAGS = $(CXX_INCLUDES) -O3 -Wall

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(CXX_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CXX_SOURCES)))

$(BUILD_DIR)/libbsli.a: $(OBJECTS)
	ar rs $(BUILD_DIR)/libbsli.a $(OBJECTS)

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	$(CXX) -c $(CXXFLAGS) $< -o $@

	
$(BUILD_DIR):
	mkdir $@		
