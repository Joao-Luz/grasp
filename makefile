TARGET := grasp_cvrp
LINK := -lm
CFLAGS := -g
INCLUDE_PATHS := -Igrasp -Icvrp
CXX := gcc
IN := cvrp/vrp-A/A-n32-k5.vrp

all: $(TARGET)

SRC = grasp/grasp.c cvrp/cvrp.c cvrp/main.c
HEADERS = grasp/grasp.h cvrp/cvrp.h 

OBJECTS := $(SRC:%.c=build/%.o)

build/grasp/%.o: grasp/%.c $(HEADERS)
	@mkdir -p build/grasp/
	$(CXX) $(INCLUDE_PATHS) $(CFLAGS) -c -o $@ $<

build/cvrp/%.o: cvrp/%.c $(HEADERS)
	@mkdir -p build/cvrp/
	$(CXX) $(INCLUDE_PATHS) $(CFLAGS) -c -o $@ $<

$(TARGET): $(OBJECTS)
	$(CXX) $(INCLUDE_PATHS) $(OBJECTS) $(LINK) -o $@

clean:
	-rm -f -r build
	-rm -f *.o
	-rm -f $(TARGET)

debug: $(TARGET)
	@valgrind --leak-check=full ./$(TARGET) $(IN)

run: $(TARGET)
	@echo -e "Running $(IN)...\n"
	@./$(TARGET) $(IN)
	@echo -e "\nOptimal solution:\n"
	@cat $(IN:%.vrp=%.sol)
