

call ..\..\..\..\..\zephyr-env.cmd
@echo on

rmdir build_network /S /Q

mkdir build_network

cd build_network

cmake -GNinja -DBOARD=nrf53_pca10095_network ..

ninja

cd ..
