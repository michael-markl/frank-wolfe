#!/bin/sh
set -eu

project_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
staging_dir=$(mktemp -d)
trap 'rm -rf "$staging_dir"' 0
trap 'exit 1' HUP INT TERM

cp "$project_dir/README.md" "$project_dir/Cargo.lock" "$project_dir/Cargo.toml" "$staging_dir/"
cp -R "$project_dir/data" "$project_dir/crates" "$project_dir/results" "$staging_dir/"
cp "$project_dir/run-experiments.sh" "$staging_dir/run-experiments.sh"
sed 's|^NETWORKS=.*$|NETWORKS="${NETWORKS-./data/TransportationNetworks}"|' \
    "$project_dir/run-experiments.sh" > "$staging_dir/run-experiments.sh"

# Clone the latest default branch (main/master), with only its latest commit.
git clone --depth 1 https://github.com/bstabler/TransportationNetworks.git \
    "$staging_dir/data/TransportationNetworks"
commit=$(git -C "$staging_dir/data/TransportationNetworks" rev-parse HEAD)

for subfolder in .git chicago-regional Sydney Philadelphia Austin GoldCoast Birmingham-England; do
    rm -rf "$staging_dir/data/TransportationNetworks/$subfolder"
done

cpu_model=$(
    case $(uname -s) in
        Linux) LC_ALL=C lscpu | sed -n 's/^Model name:[[:space:]]*//p' ;;
        Darwin) sysctl -n machdep.cpu.brand_string ;;
        *) uname -m ;;
    esac
)
os_version=$(
    case $(uname -s) in
        Linux)
            if [ -r /etc/os-release ]; then
                . /etc/os-release
                printf '%s (kernel %s)\n' "${PRETTY_NAME:-Linux}" "$(uname -r)"
            else
                uname -sr
            fi
            ;;
        Darwin) printf 'macOS %s\n' "$(sw_vers -productVersion)" ;;
        *) uname -sr ;;
    esac
)
rust_version=$(cd "$project_dir" && rustc --version)
cargo_version=$(cd "$project_dir" && cargo --version)

cat > "$staging_dir/README.md" <<EOF
# Carbon and Cordon Pricing Experiments

This archive contains the implemented Frank--Wolfe style method used to evaluate carbon and cordon pricing scenarios in real-world networks.

This README contains instructions to reproduce the experiments.

## Data

Regarding the data on real-world networks, we rely on the well-known collection "TransportationNetworks", maintained by Ben Stabler located at the following GitHub repository:

https://github.com/bstabler/TransportationNetworks

For the berlin-center network, the relevant data on the Ringbahn cordon is located at ./data/berlin-center-cordon-edge-map.csv.
It contains, for every edge, whether it is considered inside or outside the cordon.

For the sioux-falls network, the actually used network with supplied link lengths is located at ./data/sioux-falls-net-with-lengths/SiouxFalls_net.tntp.
More information is available at ./data/sioux-falls-net-with-lengths/README.md.

## Prerequisites

It should be possible to run the experiments on any modern platform (Windows, Linux, MacOS).
To compile the program, a current version of the Rust toolchain is required, see https://rust-lang.org/tools/install/.

## Running the experiments

If you are using Linux or MacOS, you should be able to run the experiments by simply executing the shell script \`run-experiments.sh\`.

The output of the experiments is provided in the "results" folder.

Note that a partial copy of the TransportationNetworks repository (commit: $commit) is located in this archive
at ./data/TransportationNetworks. Large and unused networks were stripped to reduce the size of the archive.

## Used Specifications

The results at ./results were obtained with the following system:

- CPU model: $cpu_model
- Operating system: $os_version
- Rust compiler: $rust_version
- Cargo: $cargo_version
EOF

(
    cd "$staging_dir"
    zip -qr scholar-one.zip README.md run-experiments.sh data crates results Cargo.lock Cargo.toml
)
mv "$staging_dir/scholar-one.zip" "$project_dir/scholar-one.zip"
printf 'Created %s/scholar-one.zip\n' "$project_dir"
