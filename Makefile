default:
	./build.sh

all:
	./build_all.sh

clean:
	rm -rf ./Thirdparty/DBoW2/build
	rm -rf ./Thirdparty/g2o/build
	rm -rf ./Thirdparty/Sophus/build
	rm -rf ./build
	rm -rf ./lib/*
