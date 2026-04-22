# parameters
ARG PROJECT_NAME
ARG PROJECT_DESCRIPTION
ARG PROJECT_MAINTAINER
# pick an icon from: https://fontawesome.com/v4.7.0/icons/
ARG PROJECT_ICON="cogs"
ARG PROJECT_FORMAT_VERSION

# ==================================================>
# ==> Do not change the code below this line
ARG ARCH
ARG DISTRO
ARG DOCKER_REGISTRY
ARG BASE_REPOSITORY
ARG BASE_ORGANIZATION=duckietown
ARG BASE_TAG=${DISTRO}-${ARCH}
ARG LAUNCHER=default

# define base image
FROM ${DOCKER_REGISTRY}/${BASE_ORGANIZATION}/${BASE_REPOSITORY}:${BASE_TAG} as base

# recall all arguments
ARG ARCH
ARG DISTRO
ARG DOCKER_REGISTRY
ARG PROJECT_NAME
ARG PROJECT_DESCRIPTION
ARG PROJECT_MAINTAINER
ARG PROJECT_ICON
ARG PROJECT_FORMAT_VERSION
ARG BASE_TAG
ARG BASE_REPOSITORY
ARG BASE_ORGANIZATION
ARG LAUNCHER
# - buildkit
ARG TARGETPLATFORM
ARG TARGETOS
ARG TARGETARCH
ARG TARGETVARIANT

# check build arguments
RUN dt-args-check \
    "PROJECT_NAME" "${PROJECT_NAME}" \
    "PROJECT_DESCRIPTION" "${PROJECT_DESCRIPTION}" \
    "PROJECT_MAINTAINER" "${PROJECT_MAINTAINER}" \
    "PROJECT_ICON" "${PROJECT_ICON}" \
    "PROJECT_FORMAT_VERSION" "${PROJECT_FORMAT_VERSION}" \
    "ARCH" "${ARCH}" \
    "DISTRO" "${DISTRO}" \
    "DOCKER_REGISTRY" "${DOCKER_REGISTRY}" \
    "BASE_REPOSITORY" "${BASE_REPOSITORY}" \
    && dt-check-project-format "${PROJECT_FORMAT_VERSION}"

# define/create repository path
ARG PROJECT_PATH="${SOURCE_DIR}/${PROJECT_NAME}"
ARG PROJECT_LAUNCHERS_PATH="${LAUNCHERS_DIR}/${PROJECT_NAME}"
RUN mkdir -p "${PROJECT_PATH}" "${PROJECT_LAUNCHERS_PATH}"
WORKDIR "${PROJECT_PATH}"

# keep some arguments as environment variables
ENV DT_PROJECT_NAME="${PROJECT_NAME}" \
    DT_PROJECT_DESCRIPTION="${PROJECT_DESCRIPTION}" \
    DT_PROJECT_MAINTAINER="${PROJECT_MAINTAINER}" \
    DT_PROJECT_ICON="${PROJECT_ICON}" \
    DT_PROJECT_PATH="${PROJECT_PATH}" \
    DT_PROJECT_LAUNCHERS_PATH="${PROJECT_LAUNCHERS_PATH}" \
    DT_LAUNCHER="${LAUNCHER}" \
    DT_DISTRO="${DISTRO}"

# install apt dependencies
COPY ./dependencies-apt.txt "${PROJECT_PATH}/"
RUN dt-apt-install ${PROJECT_PATH}/dependencies-apt.txt

# Debian's python3-libcamera installs into /usr/lib/<triplet>/python3.X/site-packages/,
# which Python 3.12 does not include in sys.path by default. Expose it via a .pth file
# so picamera2 can `import libcamera` at runtime.
RUN set -e; \
    pyver="$(python3 -c 'import sys;print(f"{sys.version_info.major}.{sys.version_info.minor}")')"; \
    triplet="$(python3 -c 'import sysconfig;print(sysconfig.get_config_var("MULTIARCH"))')"; \
    libcamera_dir="/usr/lib/${triplet}/python${pyver}/site-packages"; \
    if [ -d "${libcamera_dir}/libcamera" ]; then \
        echo "${libcamera_dir}" > "/usr/local/lib/python${pyver}/dist-packages/libcamera.pth"; \
    fi

# install python3 dependencies
ARG PIP_INDEX_URL="https://pypi.org/simple"
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
COPY ./dependencies-py3.* "${PROJECT_PATH}/"
RUN dt-pip3-install "${PROJECT_PATH}/dependencies-py3.*"

# picamera2's previews/__init__.py eagerly imports drm_preview, which requires
# pykms (python3-kms++). That package is not available on Ubuntu Noble and we
# run headless (no DRM preview), so guard the import to tolerate ImportError.
RUN set -e; \
    pyver="$(python3 -c 'import sys;print(f"{sys.version_info.major}.{sys.version_info.minor}")')"; \
    previews_init="/usr/local/lib/python${pyver}/dist-packages/picamera2/previews/__init__.py"; \
    if [ -f "${previews_init}" ] && ! grep -q "except ImportError" "${previews_init}"; then \
        sed -i 's|^from .drm_preview import DrmPreview$|try:\n    from .drm_preview import DrmPreview\nexcept ImportError:\n    DrmPreview = None|' "${previews_init}"; \
    fi

# copy the source code
COPY ./packages "${PROJECT_PATH}/packages"

# build packages
RUN dt-python-build ${WORKSPACE_DIR}

# install launcher scripts
COPY ./launchers/. "${PROJECT_LAUNCHERS_PATH}/"
RUN dt-install-launchers "${PROJECT_LAUNCHERS_PATH}"

# install scripts
COPY ./assets/entrypoint.d "${PROJECT_PATH}/assets/entrypoint.d"
COPY ./assets/environment.d "${PROJECT_PATH}/assets/environment.d"

# define default command
CMD ["bash", "-c", "dt-launcher-${DT_LAUNCHER}"]

# store module metadata
LABEL \
    # module info
    org.duckietown.label.project.name="${PROJECT_NAME}" \
    org.duckietown.label.project.description="${PROJECT_DESCRIPTION}" \
    org.duckietown.label.project.maintainer="${PROJECT_MAINTAINER}" \
    org.duckietown.label.project.icon="${PROJECT_ICON}" \
    org.duckietown.label.project.path="${PROJECT_PATH}" \
    org.duckietown.label.project.launchers.path="${PROJECT_LAUNCHERS_PATH}" \
    # format
    org.duckietown.label.format.version="${PROJECT_FORMAT_VERSION}" \
    # platform info
    org.duckietown.label.platform.os="${TARGETOS}" \
    org.duckietown.label.platform.architecture="${TARGETARCH}" \
    org.duckietown.label.platform.variant="${TARGETVARIANT}" \
    # code info
    org.duckietown.label.code.distro="${DISTRO}" \
    org.duckietown.label.code.launcher="${LAUNCHER}" \
    org.duckietown.label.code.python.registry="${PIP_INDEX_URL}" \
    # base info
    org.duckietown.label.base.organization="${BASE_ORGANIZATION}" \
    org.duckietown.label.base.repository="${BASE_REPOSITORY}" \
    org.duckietown.label.base.tag="${BASE_TAG}"
# <== Do not change the code above this line
# <==================================================

# force reinstall RPi.GPIO to remove nVidia's dummy RPi libraries
RUN python3 -m pip install --ignore-installed --force-reinstall RPi.GPIO

# this is necessary for the camera pipeline to work on the Jetson Nano
COPY assets/etc/ld.so.conf.d/nvidia-tegra.conf /etc/ld.so.conf.d/nvidia-tegra.conf
COPY assets/etc/ld.so.conf.d/nvidia-tegra-egl.conf /etc/ld.so.conf.d/nvidia-tegra-egl.conf
RUN ldconfig

# copy fonts (used by the text display renderer)
COPY assets/usr/share/fonts/*.ttf /usr/share/fonts/
