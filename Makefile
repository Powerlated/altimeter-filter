BUILD_DIR = build

CXX_INCLUDES = \
-Ilib \
-Ilib/imgui-1.91.9b \
-Ilib/imgui-1.91.9b/backends \
-Ilib/implot-0.16

CXX_SOURCES = \
src/airbrakes/AltimeterFilter.cpp \
src/airbrakes/airbrakes.cpp \
src/main.cpp \
src/visualizer.cpp \
lib/imgui-1.91.9b/imgui_demo.cpp \
lib/imgui-1.91.9b/imgui_draw.cpp \
lib/imgui-1.91.9b/imgui_tables.cpp \
lib/imgui-1.91.9b/imgui_widgets.cpp \
lib/imgui-1.91.9b/imgui.cpp \
lib/imgui-1.91.9b/backends/imgui_impl_sdl3.cpp \
lib/imgui-1.91.9b/backends/imgui_impl_sdlrenderer3.cpp \
lib/implot-0.16/implot_demo.cpp \
lib/implot-0.16/implot_items.cpp \
lib/implot-0.16/implot.cpp \


CXXFLAGS = $(CXX_INCLUDES) -g -O3 -Wall -std=c++23

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(CXX_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CXX_SOURCES)))

# Detect Windows environment
ifeq ($(OS),Windows_NT)
    EXE_EXT := .exe
else
    EXE_EXT :=
endif

TARGET := main$(EXE_EXT)

$(BUILD_DIR)/$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) $^ -lSDL3 -o $@

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	$(CXX) -c $(CXXFLAGS) $< -o $@

	
$(BUILD_DIR):
	mkdir $@		

.PHONY: run clean
run: $(BUILD_DIR)/$(TARGET)
	$<

clean:
	rm -r build