NETWORKS="${NETWORKS-../TransportationNetworks}"

set -x

cargo build --release

mkdir -p ./results
# PERMIT BASED CORDON PRICING 
(
    set -x
    time cargo run --release -- carbon-pricing --graph "$NETWORKS/Berlin-Center/berlin-center_net.tntp" \
        --demand "$NETWORKS/Berlin-Center/berlin-center_trips.tntp" \
        --out_csv results/berlin-center-permit-based-cordon.csv --min_price=0 --max_price=6 --min_per_time_unit=0.01666666667 \
        --max_iter  500000 \
        --km_per_distance_unit=0.001 --steps 101 --cordon_edge_map ./data/berlin-center-cordon-edge-map.csv --permit_based
) 2>&1 | tee results/berlin-center-permit-based-cordon.log

(
    set -x
    time cargo run --release -- carbon-pricing --graph "$NETWORKS/Berlin-Center/berlin-center_net.tntp" \
        --demand "$NETWORKS/Berlin-Center/berlin-center_trips.tntp" \
        --out_csv results/berlin-center-min-permits.csv --min_price=1e128 --max_price=1e128 --min_per_time_unit=0.01666666667 \
        --max_iter  500000 \
        --km_per_distance_unit=0.001 --steps 1 --cordon_edge_map ./data/berlin-center-cordon-edge-map.csv --permit_based
) 2>&1 | tee results/berlin-center-min-permits.log

# EDGE BASED CORDON PRICING
(
    set -x
    time cargo run --release -- carbon-pricing --graph "$NETWORKS/Berlin-Center/berlin-center_net.tntp" \
    --demand "$NETWORKS/Berlin-Center/berlin-center_trips.tntp" \
    --out_csv results/berlin-center-edge-based-cordon.csv \
    --max_iter  500000 \
    --min_price 0 --max_price 10 --steps 101 --min_per_time_unit 0.01666666667 --km_per_distance_unit=0.001 \
    --cordon_edge_map ./data/berlin-center-cordon-edge-map.csv
) 2>&1 | tee results/berlin-center-edge-based-cordon.log

# CARBON PRICING
(
    set -x
    time cargo run --release -- carbon-pricing --graph "$NETWORKS/Berlin-Center/berlin-center_net.tntp" \
        --demand "$NETWORKS/Berlin-Center/berlin-center_trips.tntp" \
        --out_csv results/berlin-center-carbon-pricing.csv \
        --max_iter  500000 \
        --min_price 0 --max_price 10 --steps 101 --min_per_time_unit 0.01666666667 --km_per_distance_unit=0.001
) 2>&1 | tee results/berlin-center-carbon-pricing.log

(
    set -x
    time cargo run --release -- carbon-pricing --graph "$NETWORKS/Berlin-Center/berlin-center_net.tntp" \
        --demand "$NETWORKS/Berlin-Center/berlin-center_trips.tntp" \
        --out_csv results/berlin-center-min-emission.csv \
        --max_iter  500000 \
        --min_price 1e128 --max_price 1e128 --steps 1 --min_per_time_unit 0.01666666667 --km_per_distance_unit=0.001
) 2>&1 | tee results/berlin-center-min-emission.log

(
    set -x
    time cargo run --release -- carbon-pricing --graph "$NETWORKS/Berlin-Center/berlin-center_net.tntp" \
        --demand "$NETWORKS/Berlin-Center/berlin-center_trips.tntp" \
        --out_csv results/berlin-center-min-emission-inside.csv \
        --max_iter 500000 \
        --min_price 1e128 --max_price 1e128 --steps 1 --min_per_time_unit 0.01666666667 --km_per_distance_unit=0.001 \
        --cordon_edge_map ./data/berlin-center-cordon-edge-map.csv --carbon_pricing_inside
) 2>&1 | tee results/berlin-center-min-emission-inside.log

(
    set -x
    time cargo run --release -- carbon-pricing --graph "$NETWORKS/Berlin-Friedrichshain/friedrichshain-center_net.tntp" \
    --demand "$NETWORKS/Berlin-Friedrichshain/friedrichshain-center_trips.tntp" \
    --max_iter  500000 \
    --out_csv results/berlin-friedrichshain-carbon-pricing.csv --min_price 0 --max_price 10 --steps 101 \
    --min_per_time_unit 0.01666666667 --km_per_distance_unit=0.001
) 2>&1 | tee results/berlin-friedrichshain-carbon-pricing.log

(
    set -x
    time cargo run --release -- carbon-pricing --graph "$NETWORKS/Berlin-Tiergarten/berlin-tiergarten_net.tntp" \
    --demand "$NETWORKS/Berlin-Tiergarten/berlin-tiergarten_trips.tntp" \
    --max_iter  500000 \
    --out_csv results/berlin-tiergarten-carbon-pricing.csv --min_price 0 --max_price 10 --steps 101 \
    --min_per_time_unit 0.01666666667 --km_per_distance_unit=0.001
) 2>&1 | tee results/berlin-tiergarten-carbon-pricing.log

(
    set -x
    time cargo run --release -- carbon-pricing --graph "$NETWORKS/Berlin-Prenzlauerberg-Center/berlin-prenzlauerberg-center_net.tntp" \
    --demand "$NETWORKS/Berlin-Prenzlauerberg-Center/berlin-prenzlauerberg-center_trips.tntp" \
    --max_iter  500000 \
    --out_csv results/berlin-prenzlauerberg-center-carbon-pricing.csv --min_price 0 --max_price 10 --steps 101 \
    --min_per_time_unit 0.01666666667 --km_per_distance_unit=0.001
) 2>&1 | tee results/berlin-prenzlauerberg-center-carbon-pricing.log


(
    set -x
    time cargo run --release -- carbon-pricing --graph "$NETWORKS/Anaheim/Anaheim_net.tntp" \
    --demand "$NETWORKS/Anaheim/Anaheim_trips.tntp" --max_iter  500000 \
    --out_csv results/anaheim-carbon-pricing.csv \
    --min_price 0 --max_price 10 --steps 101 --km_per_distance_unit=0.0003048
) 2>&1 | tee results/anaheim-carbon-pricing.log

(
    set -x
    time cargo run --release -- carbon-pricing --graph "$NETWORKS/Winnipeg/Winnipeg_net.tntp" \
    --demand "$NETWORKS/Winnipeg/Winnipeg_trips.tntp" --max_iter 500000 \
    --out_csv results/winnipeg-carbon-pricing.csv \
    --min_price 0 --max_price 100 --steps 101
) 2>&1 | tee results/winnipeg-carbon-pricing.log

(
    set -x
    time cargo run --release -- carbon-pricing --graph "./data/sioux-falls-net-with-lengths/SiouxFalls_net.tntp" \
    --demand "$NETWORKS/SiouxFalls/SiouxFalls_trips.tntp" --max_iter  500000 \
    --out_csv results/sioux-falls-carbon-pricing.csv \
    --min_price 0 --max_price 100 --steps 101 --km_per_distance_unit 1.60934 --min_per_time_unit 0.6
) 2>&1 | tee results/sioux-falls-carbon-pricing.log

(
    set -x
    time cargo run --release -- carbon-pricing --graph "./data/sioux-falls-net-with-lengths/SiouxFalls_net.tntp" \
    --demand "$NETWORKS/SiouxFalls/SiouxFalls_trips.tntp" --max_iter  500000 \
    --out_csv results/sioux-falls-min-emission.csv \
    --min_price 1e128 --max_price 1e128 --steps 1 --km_per_distance_unit 1.60934 --min_per_time_unit 0.6
) 2>&1 | tee results/sioux-falls-min-emission.log

(
    set -x
    time cargo run --release -- carbon-pricing --graph "./data/non-convex-braess/non-convex-braess_net.tntp" \
    --demand "./data/non-convex-braess/non-convex-braess_trips.tntp" --max_iter  500000 \
    --out_csv results/non-convex-braess-carbon-pricing.csv \
    --min_price 0 --max_price 20 --steps 101
) 2>&1 | tee results/non-convex-braess-carbon-pricing.log

(
    set -x
    time cargo run --release -- carbon-pricing \
    --graph ./data/non-monotone-braess/non-monotone-braess_net.tntp \
    --demand ./data/non-monotone-braess/non-monotone-braess_trips.tntp \
    --out_csv results/non-monotone-braess-carbon-pricing.csv \
    --min_price 0 --max_price 0.6666666666666667 --steps 101 --rel_gap 1e-9
) 2>&1 | tee results/non-monotone-braess-carbon-pricing.log
