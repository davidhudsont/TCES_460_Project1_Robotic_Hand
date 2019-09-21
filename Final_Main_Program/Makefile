all:
	protoc --cpp_out=. Hand.proto
	c++ glove_client.cc  Hand.pb.cc -o glove_client `pkg-config --cflags --libs protobuf` -lwiringPi -llsm9ds1
	c++ hand_server.cc  Hand.pb.cc -o hand_server `pkg-config --cflags --libs protobuf` -lwiringPi 
	g++ -o hand hand.cpp -lwiringPi
#	g++ -o glove glove.cpp -lwiringPi -llsm9ds1
#	protoc --cpp_out=. Employee.proto
#	g++ -g -Wall server.cc Employee.pb.cc -o server -lprotobuf
#	g++ -g -Wall client.cc Employee.pb.cc -o client -lprotobuf 


clean:
	rm server client Hand.pb.cc Hand.pb.h

