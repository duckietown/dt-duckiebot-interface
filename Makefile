# DO NOT MODIFY - it is auto written from duckietown-env-developer

default_arch=arm32v7
arch=$(default_arch)

branch=$(shell git rev-parse --abbrev-ref HEAD)

# name of the repo
repo=$(shell basename -s .git `git config --get remote.origin.url`)

default_tag=duckietown/$(repo):$(branch)
tag=duckietown/$(repo):$(branch)-$(arch)

labels=$(shell ./labels.py)

build: no_cache=0
build-no-cache: no_cache=1

build build-no-cache:
	docker build \
		--pull \
		$(labels) \
		-t $(tag) \
		--build-arg ARCH=$(arch) \
		--no-cache=$(no_cache) \
		.

	@if [ "$(arch)" = "$(default_arch)" ]; then \
		echo "Tagging image $(tag) as $(default_tag)."; \
		docker tag $(tag) $(default_tag); \
		echo "Done!"; \
	else \
		echo "Tagging image $(tag) as $(default_tag)-no-arm."; \
		docker tag $(tag) $(default_tag)-no-arm; \
		echo "Done!"; \
	fi

push:
	docker push $(tag)

	@if [ "$(arch)" = "$(default_arch)" ]; then \
		docker push $(default_tag); \
	else \
		docker push $(default_tag)-no-arm; \
	fi
