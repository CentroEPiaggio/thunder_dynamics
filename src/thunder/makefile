CC = g++
LFLAGS = -lcasadi -lyaml-cpp
CFLAGS = -std=c++17
MAIN = main_Thunder_test
MOD1 = RobKinBasic
MOD2 = RobKinAdv
MOD3 = RobReg
MOD4 = FrameOffset
MOD5 = RobDyn
MOD_I = CasadiObj
SUB_LIB = library/
SUB_SRC = src/


$(MAIN): $(MAIN).o $(MOD1).o $(MOD2).o $(MOD3).o $(MOD4).o $(MOD5).o
	$(CC) $(CFLAGS) -o $(MAIN) $(MAIN).o $(MOD1).o $(MOD2).o $(MOD3).o $(MOD4).o $(MOD5).o -L/usr/local/lib $(LFLAGS)

$(MAIN).o: $(MAIN).cpp $(SUB_LIB)$(MOD1).h $(SUB_LIB)$(MOD2).h $(SUB_LIB)$(MOD3).h $(SUB_LIB)$(MOD4).h $(SUB_LIB)$(MOD5).h $(SUB_LIB)$(MOD_I).h
	$(CC) $(CFLAGS) -c $(MAIN).cpp $(LFLAGS)

$(MOD1).o: $(SUB_SRC)$(MOD1).cpp $(SUB_LIB)$(MOD1).h
	$(CC) $(CFLAGS) -c $(SUB_SRC)$(MOD1).cpp $(LFLAGS)

$(MOD2).o: $(SUB_SRC)$(MOD2).cpp $(SUB_LIB)$(MOD2).h
	$(CC) $(CFLAGS) -c $(SUB_SRC)$(MOD2).cpp $(LFLAGS)

$(MOD3).o: $(SUB_SRC)$(MOD3).cpp $(SUB_LIB)$(MOD3).h
	$(CC) $(CFLAGS) -c $(SUB_SRC)$(MOD3).cpp $(LFLAGS)

$(MOD4).o: $(SUB_SRC)$(MOD4).cpp $(SUB_LIB)$(MOD4).h
	$(CC) $(CFLAGS) -c $(SUB_SRC)$(MOD4).cpp $(LFLAGS)

$(MOD5).o: $(SUB_SRC)$(MOD5).cpp $(SUB_LIB)$(MOD5).h
	$(CC) $(CFLAGS) -c $(SUB_SRC)$(MOD5).cpp $(LFLAGS)

clean:
	rm -f $(MAIN).o $(MOD1).o $(MOD2).o $(MOD3).o $(MOD4).o $(MOD5).o $(MOD6).o $(MAIN)
