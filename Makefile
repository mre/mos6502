.PHONY: help
help: ## Show this help.
	@awk 'BEGIN {FS = ":.*?## "; printf "\nUsage:\n  make \033[36m<target>\033[0m\n"} /^[a-zA-Z_-]+( +[a-zA-Z_-]+)*:.*?## / { printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2 } /^##@/ { printf "\n\033[1m%s\033[0m\n", substr($$0, 5) } ' $(MAKEFILE_LIST)

# Test assembly binaries
TEST_ASSETS = tests/assets
INTERRUPT_TEST_SRC = $(TEST_ASSETS)/6502_interrupt_test.s
INTERRUPT_TEST_BIN = $(TEST_ASSETS)/6502_interrupt_test.bin
LINKER_CFG = $(TEST_ASSETS)/linker.cfg

.PHONY: build-test-bins
build-test-bins: $(INTERRUPT_TEST_BIN) ## Build test assembly binaries

$(INTERRUPT_TEST_BIN): $(INTERRUPT_TEST_SRC) $(LINKER_CFG)
	@echo "Building interrupt test binary..."
	ca65 -o $(INTERRUPT_TEST_SRC:.s=.o) $(INTERRUPT_TEST_SRC)
	ld65 -C $(LINKER_CFG) -o $(INTERRUPT_TEST_BIN) $(INTERRUPT_TEST_SRC:.s=.o)
	@rm -f $(INTERRUPT_TEST_SRC:.s=.o)

.PHONY: test
test: build-test-bins ## Run tests
	cargo test

.PHONY: clean
clean: ## Clean up
	cargo clean
	
.PHONY: format fmt
format fmt: ## Format code
	cargo fmt

.PHONY: lint
lint: ## Run linter
	cargo clippy --all-targets --all-features -- -D warnings

.PHONY: lint-fix
lint-fix: ## Run linter; apply fixes
	cargo clippy --all-targets --all-features --allow-dirty --fix -- -D warnings