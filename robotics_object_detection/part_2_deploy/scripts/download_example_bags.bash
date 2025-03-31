mkdir -p /robotics1/data
cd /robotics1/data
wget "https://chmura.put.poznan.pl/s/OtTAXP3EkZvrIxl/download" -O bag_car_chase.zip
wget "https://chmura.put.poznan.pl/s/QHK2yhs1UobmcFX/download" -O bag_overpass.zip
unzip bag_car_chase.zip
unzip bag_overpass.zip
rm bag_car_chase.zip
rm bag_overpass.zip
cd /robotics1
