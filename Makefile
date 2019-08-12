VERSION=1

root_dir:=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

build_mk=$(root_dir)/docker-tools/build.mk

env_check:
	@# check if the docker-tools repo is initialized
	@if [ ! -f "$(build_mk)" ]; then \
		echo "The submodule 'docker-tools' is not initialized. Initializing now..."; \
		git submodule init; \
		git submodule update; \
	fi

build push: env_check
	@$(MAKE) -f $(build_mk) _$@

build-all:
	@$(MAKE) build arch=amd64
	@$(MAKE) build arch=arm32v7

push-all:
	@$(MAKE) push arch=amd64
	@$(MAKE) push arch=arm32v7
