CC=gcc
CFLAGS=-c
VALGRIND_FLAGS=

ifdef DEBUG
	CFLAGS += -DDEBUG=1
	VALGRIND_FLAGS += -g -O0
endif

OBJECTS_FOLDER=objects
BUILD_FOLDER=build
SOURCE_FOLDER=src
EXAMPLES_FOLDER=examples

SOURCES=adxl355.c
OBJECTS=$(foreach obj, $(SOURCES:.c=.o), $(OBJECTS_FOLDER)/$(obj))
EXAMPLES=reset
EXAMPLES_SOURCE=$(foreach example,$(EXAMPLES),$(example).c)

LIB=rpiadxl355.a

all: $(OBJECTS_FOLDER) $(LIB) examples 

$(LIB): $(OBJECTS)
	ar rcs $@ $(OBJECTS)

$(OBJECTS_FOLDER)/%.o: $(SOURCE_FOLDER)/%.c
	$(CC) $(CFLAGS) -o $@ $<

examples: $(EXAMPLES)

$(EXAMPLES): $(BUILD_FOLDER)
	$(CC) $(VALGRIND_FLAGS) -I$(SOURCE_FOLDER) $(EXAMPLES_FOLDER)/$@.c -l:$(LIB) -lwiringPi -L./ -o $(BUILD_FOLDER)/$@

$(BUILD_FOLDER):
	mkdir $(BUILD_FOLDER)

$(OBJECTS_FOLDER):
	mkdir $(OBJECTS_FOLDER)

clean_examples:
	rm -rf $(BUILD_FOLDER)

clean:
	rm -rf *.o
	rm -rf $(OBJECTS_FOLDER)