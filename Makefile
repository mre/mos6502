.PHONY: help
help: ## Show this help.
	@awk 'BEGIN {FS = ":.*?## "; printf "\nUsage:\n  make \033[36m<target>\033[0m\n"} /^[a-zA-Z_-]+:.*?## / { printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2 } /^##@/ { printf "\n\033[1m%s\033[0m\n", substr($$0, 5) } ' $(MAKEFILE_LIST)

.PHONY: test
test: ## Run tests
	cargo test

.PHONY: clean
clean: ## Clean up
	cargo clean
	
.PHONY: fmt format
fmt: ## Format code
	cargo fmt

format: fmt

.PHONY: lint
lint: ## Run linter
	cargo clippy --all-targets --all-features -- -D warnings

.PHONY: lint-fix
lint-fix: ## Run linter; apply fixes
	cargo clippy --all-targets --all-features --allow-dirty --fix -- -D warnings