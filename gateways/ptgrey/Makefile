COMPILER = g++
FLAGS = -std=c++14 -O3 -Wall -Werror -Wextra

SO_DEPS = $(shell pkg-config --libs --cflags libSimpleAmqpClient msgpack librabbitmq opencv)
SO_DEPS += -lboost_program_options -lpthread -lflycapture

TARGET=ptgrey
SERVICE=gateway-ptgrey
VERSION=latest
DOCKER_USER=mendonca

all: ptgrey

clean:
	rm ptgrey

ptgrey: ptgrey.cpp 
	$(COMPILER) $^ -o $@ $(FLAGS) $(SO_DEPS)

docker:
	rm -rf libs/
	mkdir libs/
	lddcp $(TARGET) libs/
	docker build -t $(DOCKER_USER)/$(SERVICE):$(VERSION) .
	rm -rf libs/
