# to run: ./test.sh 2> /dev/null
tests=(TEST1 TEST2 TEST3 TEST4 TEST5 TEST6 TEST7 TEST8 TEST9 TEST10 TEST11 TEST12)
#tests=(TEST1)
for i in "${tests[@]}"
do
   echo "$i is running!"
   rm *.vvp
   cd ../sw/
   gcc -D$i -o native.exe main.c
   ref=$(./native.exe | awk '{$1=$1;print}')
   ./build.sh main.c  $i 
   
   cd ../dv/
   iverilog -DTEST_FILE="../sw/main.hex" -Wall -o ../dv/n5_tb.vvp -s n5_tb  -c dv_files.list 
   ./n5_tb.vvp > n5_tb.log
   res=$(grep "RF\[10\]" n5_tb.log | tail -1 | cut -d"(" -f2 | cut -d")" -f1 | awk '{$1=$1;print}')
   if [ $ref = $res ]
        then
                echo "$i Passed"
                grep "CPI" n5_tb.log | tail -1 | cut -d"(" -f2 | cut -d")" -f1 | awk '{$1=$1;print}'
        else
                echo "$i Failed"
                echo "$res should be $ref"
   fi
   mv n5_tb.vcd n5_tb_$i.vcd
   echo "=-=-=-=-=-="
   # do whatever on $i
done