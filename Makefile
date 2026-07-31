BINARY := camera-calibration
BIN_DIR := bin

.PHONY: setup build lint check-lint test module.tar.gz clean

default: module.tar.gz

setup:
	go mod tidy

build:
	@mkdir -p $(BIN_DIR)
	go build -o $(BIN_DIR)/$(BINARY) ./cmd/module
	@echo "Binary: $(BIN_DIR)/$(BINARY)"

lint:
	golangci-lint run --fix ./...

check-lint:
	golangci-lint run ./...

test:
	go test ./...

module.tar.gz: build
	tar czf module.tar.gz $(BIN_DIR)/$(BINARY) meta.json
	@echo "Created module.tar.gz"

clean:
	rm -rf $(BIN_DIR) module.tar.gz
	@echo "Clean complete"
