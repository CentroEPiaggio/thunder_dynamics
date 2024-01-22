CC = g++
LFLAGS = -lcasadi -lyaml-cpp
CFLAGS = -std=c++17
MAIN = main_YAML
MOD1 = urdf2dh_inertial
SUB_LIB = library/
SUB_SRC = src/


$(MAIN): $(MAIN).o $(MOD1).o
	$(CC) $(CFLAGS) -o $(MAIN) $(MAIN).o $(MOD1).o -L/usr/local/lib $(LFLAGS)

$(MAIN).o: $(MAIN).cpp $(SUB_LIB)$(MOD1).h
	$(CC) $(CFLAGS) -c $(MAIN).cpp $(LFLAGS)

$(MOD1).o: $(SUB_SRC)$(MOD1).cpp $(SUB_LIB)$(MOD1).h
	$(CC) $(CFLAGS) -c $(SUB_SRC)$(MOD1).cpp $(LFLAGS)

clean:
	rm -f $(MAIN).o $(MOD1).o $(MAIN)
