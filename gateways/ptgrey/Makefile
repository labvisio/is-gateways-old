COMPILER = g++
FLAGS = -std=c++14 -O3 -Wall -Werror -Wextra

SO_DEPS = $(shell pkg-config --libs --cflags libSimpleAmqpClient msgpack librabbitmq opencv theoradec theoraenc)
SO_DEPS += -lboost_program_options -lpthread -lflycapture

TARGET=ptgrey
SERVICE=gateway-ptgrey
VERSION=0.0.3
DOCKER_USER=mendonca

all: $(TARGET)

clean:
	rm $(TARGET)

$(TARGET): $(TARGET).cpp 
	$(COMPILER) $^ -o $@ $(FLAGS) $(SO_DEPS)

docker: $(TARGET)
	rm -rf libs/
	mkdir libs/
	lddcp $(TARGET) libs/
	docker build -t $(DOCKER_USER)/$(SERVICE):$(VERSION) .
	rm -rf libs/

docker_push: docker
	docker image push $(DOCKER_USER)/$(SERVICE):$(VERSION)