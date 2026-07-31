BINARY := camera-calibration
BIN_DIR := bin

.PHONY: setup venv build lint check-lint test python-test module.tar.gz clean

default: module.tar.gz

setup: venv
	go mod tidy

venv:
	bash ./setup_python.sh

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
	$(MAKE) python-test

python-test:
	.venv/bin/pytest python/ -v

module.tar.gz: build
	tar czf module.tar.gz $(BIN_DIR)/$(BINARY) meta.json first_run.sh python requirements.txt
	@echo "Created module.tar.gz"

clean:
	rm -rf $(BIN_DIR) module.tar.gz
	@echo "Clean complete"
