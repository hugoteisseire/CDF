# Makefile pour le simulateur LIDAR

CC = gcc
CFLAGS = -Wall -O2
LDFLAGS =

TARGET = test_lidar_socket
SRC = test_lidar_socket.c

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRC) $(LDFLAGS)
	@echo "✅ Compilation réussie: ./$(TARGET)"

clean:
	rm -f $(TARGET)
	rm -f /tmp/robot.sock
	@echo "🧹 Nettoyage effectué"

run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run
