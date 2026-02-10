NETWORKS="../TransportationNetworks"

set -x

mkdir -p ./results
if [ -n "1" ]; then
    # PERMIT BASED CORDON PRICING 
    (
        set -x
        time cargo run --release -- carbon-pricing --graph "$NETWORKS/Berlin-Center/berlin-center_net.tntp" \
            --demand "$NETWORKS/Berlin-Center/berlin-center_trips.tntp" \
            --out results/berlin-center-permit-based-cordon.csv --min_price=0 --max_price=10 --min_per_time_unit=0.01666666667 \
            --km_per_distance_unit=0.001 --steps 101 --cordon_edge_map ./berlin-center-cordon-edge-map.csv --permit_based 2>&1 | tee results/berlin-center-permit-based-cordon.log
    )

    # EDGE BASED CORDON PRICING
    (
        set -x
        time cargo run --release -- carbon-pricing --graph "$NETWORKS/Berlin-Center/berlin-center_net.tntp" \
        --demand "$NETWORKS/Berlin-Center/berlin-center_trips.tntp" --out results/berlin-center-edge-based-cordon.csv \
        --min_price 0 --max_price 10 --steps 101 --min_per_time_unit 0.01666666667 --km_per_distance_unit=0.001 \
        --cordon_edge_map ./berlin-center-cordon-edge-map.csv 2>&1 | tee results/berlin-center-edge-based-cordon.log
    )
fi

# CARBON PRICING
(
    set -x
    time cargo run --release -- carbon-pricing --graph "$NETWORKS/Berlin-Center/berlin-center_net.tntp" \
        --demand "$NETWORKS/Berlin-Center/berlin-center_trips.tntp" --out results/berlin-center-carbon-pricing.csv \
        --min_price 0 --max_price 10 --steps 101 --min_per_time_unit 0.01666666667 --km_per_distance_unit=0.001 2>&1 | tee results/berlin-center-carbon-pricing.log
)

(
    set -x
    time cargo run --release -- carbon-pricing --graph "$NETWORKS/Berlin-Friedrichshain/friedrichshain-center_net.tntp" \
    --demand "$NETWORKS/Berlin-Friedrichshain/friedrichshain-center_trips.tntp" \
    --out results/berlin-friedrichshain-carbon-pricing.csv --min_price 0 --max_price 10 --steps 101 \
    --min_per_time_unit 0.01666666667 --km_per_distance_unit=0.001 2>&1 | tee results/berlin-friedrichshain-carbon-pricing.log
)
(
    set -x
    time cargo run --release -- carbon-pricing --graph "$NETWORKS/Berlin-Tiergarten/berlin-tiergarten_net.tntp" \
    --demand "$NETWORKS/Berlin-Tiergarten/berlin-tiergarten_trips.tntp" \
    --out results/berlin-tiergarten-carbon-pricing.csv --min_price 0 --max_price 10 --steps 101 \
    --min_per_time_unit 0.01666666667 --km_per_distance_unit=0.001 2>&1 | tee results/berlin-tiergarten-carbon-pricing.log
)
(
    set -x
    time cargo run --release -- carbon-pricing --graph "$NETWORKS/Berlin-Prenzlauerberg-Center/berlin-prenzlauerberg-center_net.tntp" \
    --demand "$NETWORKS/Berlin-Prenzlauerberg-Center/berlin-prenzlauerberg-center_trips.tntp" \
    --out results/berlin-prenzlauerberg-center-carbon-pricing.csv --min_price 0 --max_price 10 --steps 101 \
    --min_per_time_unit 0.01666666667 --km_per_distance_unit=0.001 2>&1 | tee results/berlin-prenzlauerberg-center-carbon-pricing.log
)

(
    set -x
    time cargo run --release -- carbon-pricing --graph "$NETWORKS/Anaheim/Anaheim_net.tntp" \
    --demand "$NETWORKS/Anaheim/Anaheim_trips.tntp" --out results/anaheim-carbon-pricing.csv \
    --min_price 0 --max_price 10 --steps 101 --km_per_distance_unit=0.0003048 2>&1 | tee results/anaheim-carbon-pricing.log
)
(
    set -x
    time cargo run --release -- carbon-pricing --graph "$NETWORKS/Winnipeg/Winnipeg_net.tntp" \
    --demand "$NETWORKS/Winnipeg/Winnipeg_trips.tntp" --out results/winnipeg-carbon-pricing.csv \
    --min_price 0 --max_price 100 --steps 101 2>&1 | tee results/winnipeg-carbon-pricing.log
)
(
    set -x
    time cargo run --release -- carbon-pricing --graph "$NETWORKS/SiouxFalls/SiouxFalls_net.tntp" \
    --demand "$NETWORKS/SiouxFalls/SiouxFalls_trips.tntp" --out results/sioux-falls-carbon-pricing.csv \
    --min_price 0 --max_price 100 --steps 101 --km_per_distance_unit=0.001 --min_per_time_unit 0.01666666667 2>&1 | tee results/sioux-falls-carbon-pricing.log
)
