SHELL := /bin/bash

COLOR_RED := \033[1;31m
COLOR_GREEN := \033[1;32m
COLOR_YELLOW := \033[1;33m
COLOR_RESET := \033[0m

all:
	@mkdir -p build
	@echo ""
	@echo "###################################################"
	@echo "# CMake"
	@echo "###################################################"
	@echo ""
	@cd build && cmake ..
	@echo ""
	@echo "###################################################"
	@echo "# Build"
	@echo "###################################################"
	@echo ""
	@cd build && make --no-print-directory -j4

clean:
	@rm -r build

tgz:
	@echo "###################################################"
	@echo "# Make x86_libs"
	@echo "###################################################"
	@echo ""
	@mkdir -p build
	@cd build; \
	cmake .. \
		-DCMAKE_TOOLCHAIN_FILE=./cmake/cross_toolchains/x86-toolchain-file.cmake \
		-DCMAKE_BUILD_TYPE="Release";
	make --no-print-directory;
	@echo "GENERATE OVER!"

tgz_xavier:
	@echo "###################################################"
	@echo "# Make xavier_libs"
	@echo "###################################################"
	@echo ""
	@sudo bash /usr/local/ADBuildTools/v0.0.1/toolchains/xavier/set_xavier_cross_compiling_env.bash
	@mkdir -p build
	@cd build; \
	cmake .. \
		-DCMAKE_TOOLCHAIN_FILE=./cmake/cross_toolchains/xavier-toolchain-file.cmake \
		-DCMAKE_BUILD_TYPE="Release" \
		-DWITH_XAVIER=ON;
	make --no-print-directory;
	@sudo rm -rf /home/sensetime/ws/jetson
	@sudo rm -rf /usr/local/cuda
	@sudo ln -s /usr/local/cuda-11.1 /usr/local/cuda
	@echo "GENERATE OVER!"

tgz_mdc:
	@echo "###################################################"
	@echo "# Make mdc_libs"
	@echo "###################################################"
	@mkdir -p build
	@cd build; \
	cmake .. \
		-DCMAKE_TOOLCHAIN_FILE=./cmake/cross_toolchains/mdc-toolchain-file.cmake \
		-DCMAKE_BUILD_TYPE="Release" \
		-DCMAKE_INSTALL_PREFIX="/opt/senseauto/tmp/senseauto-pilot-planning" \
		-DBUILD_MDC=ON \
		-DWITH_MDC=ON \
		-DCATKIN_BUILD_BINARY_PACKAGE="1" \
		-DMODULE_VERSION="$(TGZ_VER)$(CLEAN_VER)"; \
	make --no-print-directory -j4
	@sudo rm -rf /home/sensetime/ws/jetson

test:
	@echo "###################################################"
	@echo "# Test"
	@echo "###################################################"
	@echo ""
	@mkdir -p build
	@cd build; \
	make test