all: cf_display

CMAKE_FLAGS := -DCMAKE_BUILD_TYPE=Release

install:
	cd build && make install

uninstall:
	cd build && make uninstall

cf_display:
	@mkdir -p build
	cd build && cmake $(CMAKE_FLAGS) ..
ifneq ($(MAKE),)
	cd build && $(MAKE)
else
	cd build && make
endif

.PHONY: clean
clean:
	rm -rf build
