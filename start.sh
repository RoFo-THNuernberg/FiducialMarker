echo "startet Roboter-Detection"

cd reacTIVision/linux
make
make run &

echo "reacTIVision wurde gestartet"

cd ../..
make
make run

echo "OpenFramework App wurde gestartet"


