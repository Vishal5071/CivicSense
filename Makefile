TARGET = road_optimizer
CC = g++
CFLAGS = -Wall -Wextra -g -std=c++17
SOURCES = include/pugixml.cpp src/c++/main.cpp

OBJ_DIR = object

OBJECTS = $(addprefix $(OBJ_DIR)/, $(notdir $(SOURCES:.cpp=.o)))

define find_source
$(filter %/$(basename $1).cpp, $(SOURCES))
endef

define compile_rule
$(1): $(2) | $(OBJ_DIR)
	@echo "Compiling $(2) -> $(1)"
	$(CC) $(CFLAGS) -c $(2) -o $(1)
endef

$(foreach obj, $(OBJECTS), \
    $(eval $(call compile_rule, $(obj), $(call find_source, $(notdir $(obj))))) \
)

.PHONY: all clean run

$(OBJ_DIR):
	@mkdir -p $(OBJ_DIR)

all: $(TARGET)

$(TARGET): $(OBJECTS) | $(OBJ_DIR)
	$(CC) $(CFLAGS) $(OBJECTS) -o $(TARGET) -mconsole

clean:
	rm -f $(OBJECTS)
	rm -f $(TARGET)

run: $(TARGET)
	./$(TARGET)
