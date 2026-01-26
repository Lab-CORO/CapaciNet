#!/bin/bash
# set -e désactivé pour voir les erreurs au lieu de fermer

# Utilise /home qui a le plus d'espace
WORKDIR=/home/coro/apptainer_build
mkdir -p ${WORKDIR}

# Rediriger TOUT apptainer vers /home
export APPTAINER_CACHEDIR=${WORKDIR}/cache
export APPTAINER_TMPDIR=${WORKDIR}/tmp
export SINGULARITY_CACHEDIR=${WORKDIR}/cache
export SINGULARITY_TMPDIR=${WORKDIR}/tmp
export TMPDIR=${WORKDIR}/tmp

mkdir -p ${APPTAINER_CACHEDIR}
mkdir -p ${APPTAINER_TMPDIR}

docker build -t data_generation:a100 -f x86.dockerfile --build-arg PLATFORM=a100 .
echo "=== Creation de l'image docker ==="

# Debug: taille de l'image Docker
echo "=== Taille de l'image Docker ==="
docker images data_generation:a100 --format "{{.Size}}"

# Debug: taille virtuelle (décompressée) - c'est ~la taille du .tar
IMAGE_SIZE=$(docker image inspect data_generation:a100 --format '{{.Size}}')
IMAGE_SIZE_GB=$(echo "scale=2; $IMAGE_SIZE / 1024 / 1024 / 1024" | bc)
echo "Taille estimée du fichier .tar: ${IMAGE_SIZE_GB} Go"

# Debug: espace disponible
echo "=== Espace disponible dans ${WORKDIR} ==="
df -h "${WORKDIR}"

# Estimation: apptainer crée des fichiers temporaires même avec docker-daemon://
# Il faut ~2x la taille (couches décompressées + sif)
REQUIRED=$(echo "scale=2; $IMAGE_SIZE_GB * 2" | bc)
echo "=== Espace recommandé: ~${REQUIRED} Go ==="

read -p "Continuer ? (o/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Oo]$ ]]; then
    echo "Abandon. Appuie sur une touche pour fermer."
    read -n 1
else

# Nettoie le cache avant de commencer
rm -rf ${WORKDIR}/*

# Utilise docker-daemon:// pour éviter de créer un .tar intermédiaire
# --disable-cache évite de garder les couches en cache (économise de l'espace)
echo "=== Construction directe depuis Docker (sans cache) ==="
apptainer build --fakeroot --disable-cache --tmpdir ${WORKDIR} ${WORKDIR}/data_generation.sif docker-daemon://data_generation:a100

docker image rm data_generation:a100

echo "=== Taille finale du .sif ==="
ls -lh ${WORKDIR}/data_generation.sif

# Déplace le .sif vers le répertoire d'origine
ORIG_DIR=$(dirname "$(realpath "${BASH_SOURCE[0]}")")
mv ${WORKDIR}/data_generation.sif ${ORIG_DIR}/
echo "=== Fichier déplacé vers ${ORIG_DIR}/data_generation.sif ==="

fi  # fin du else (Continuer)

echo "=== Terminé. Appuie sur une touche pour fermer ==="
read -n 1

