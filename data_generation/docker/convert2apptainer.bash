docker build -t data_generation:a100 -f x86.dockerfile --build-arg PLATFORM=a100 . 
docker save data_generation:a100 -o data_generation.tar
docker image rm your-tag-name
apptainer build --fakeroot data_generation.sif docker-archive://data_generation.tar
# rm your-tarball-name.tar
