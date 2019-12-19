

call ..\..\..\..\..\zephyr-env.cmd
@echo on

rmdir build_app /S /Q

mkdir build_app

cd build_app

cmake -GNinja -DBOARD=nrf53_pca10095 ..

ninja

cd ..
