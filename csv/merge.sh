rm -f result_l
rm -f result_r
rm -f result_f

header="dist,L90,L45_3,L45_2,L45,F,R45,R45_2,R45_3,R90"

echo "$header"> result_l.csv
echo "$header"> result_r.csv
echo "$header"> result_f.csv

cat *.csv | head -n 1 > result && find -name "l_*.csv" -exec sed -e '1d' {} \; >> result_l.csv
cat *.csv | head -n 1 > result && find -name "r_*.csv" -exec sed -e '1d' {} \; >> result_r.csv
cat *.csv | head -n 1 > result && find -name "f_*.csv" -exec sed -e '1d' {} \; >> result_f.csv