SHELL := bash
.ONESHELL:
.SHELLFLAGS := -eu -o pipefail -c
.DELETE_ON_ERROR:
MAKEFLAGS += --warn-undefined-variables
MAKEFLAGS += --no-builtin-rules
MAKEFLAGS += --no-silent

default: build

src_files  := $(wildcard src/**/*.cpp)
header_files  := $(wildcard include/**/*.hpp)

build: ${src_files} ${header_files}
	catkin bt

format: clang-format

clang-format:
#	git ls-files -- '*.cpp' '*.h' '*.hpp' | xargs clang-format -i -style=./.clang-format
	git ls-files -- '*.cpp' '*.h' '*.hpp' | xargs clang-format -i

lint: clang-tidy shellharden

clang-tidy:
	git ls-files -- '*.cpp' '*.h' '*.hpp' | xargs clang-tidy

shellharden:
	git ls-files -- '*.sh' '*.bash' | xargs shellharden

dev-setup:
	./scripts/dev-setup.sh

.PHONY: default build format clang-format clang-tidy lint shellharden dev-setup
