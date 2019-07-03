READ ME FOR CHACHA20 ALGORITHM

1. Open the project and open "tb_chacha.v".
2. There is a task named "run_test_vector" which is been called at the end of the testbench. 
3. You can enter the 256-bit Key, 96-bit Nonce, Number of Rounds and 512-bit Input data in there.
4. There are various functions/tasks written to display the data, and for various functionalities which are required in the code.
5. Various local parameters are defined to represent the various states used in the algorithm.
6. The module instantiation is been done in which various input and output parameters have been passed.
7. After module instantiation, the control for the required functionality goes to file named "chacha.v".
8. In this file, various local parameters have been declared to represent the states, various registers to represent the flags used and also various registers and wires which are required.
9. "chacha_core.v" module has been instantiated in this file. The inputs passedin this file are controlled by 2 always blocks.
10. 1st always block runs on every posititve edge of clock. Main function of this block is to update the various registers and also update the value of flags. The 2nd block runs considering all the variables in that block to be in sensitivity list i.e. it'll function as a combinational logic.
11. Based on the status of various control flags and address which is been passed, the flag values are modified which in turn modies the value in the registers and then these values of registers are passed in the "chacha_core.v" module.
12. In "chacha_core.v" module the constants used to form the state matrix are declared. One can change them as per required in the algorithm. 
13. A function named "12b" is defined to convert the 32bit Hex pattern from big endian to little endian.
14. The "chacha_qr.v" module is instantiated which carries out the Quarterround calculation required in the algorithm.
15. In first always block, the flag states are being modified based on the other control flags.
16. In 2nd always block, the initial state matrix is formed.
17. In 3rd always block, the values to be passed in the quarterround module are initialized with zero and then based on various control flag status, the values are passsed in "chacha_qr.v" module and also are copied to a temporary state matrix.
18. Now the states which are obtained are in Big Endian form and need to be converted to little endian after adding them with the initial state matrix and this is donw in another always block. After adding them and converting to little endian, this state is then XORed with input data to be encrypted.
19. In another always block the quarterround counter is incremented based on various flags which drives the methodology of quarterround,i.e. first column rounds and then diagonal rounds.
20. There're two other always blocks which increments the quarterround counter and round counter respectively.
21. And then there is the main always block which controls the states driving the code.
22. And as per these control states the flags are modified which then carry out other operations.
23. Various wires and registers are declared in "chacha_qr.v" module which are used in the execution of the code.
24. The steps involved in the quarterround are implemented inside the always block involving a 32-bit addition, 32-bit xor operation and a circular left shift operation by 16,12,8 and 7 bits respectively.
25. This values are then returned to the top module.
26. To change the bit size to be XORed, go to "chacha_core.v" module and go to the always block in which the conversion between Big Endian to Little Endian is done. In this block, uncomment/comment the block below "block_state" based on the required architecture.

Steps for Reading and Exporting file in MATLAB.
I. Steps to get input file for code:
1. Load the database using command "a = load("name.mat")";
2. Convert the struct obtained into matrix using "a = cell2mat(struct2cell(a));".
3. Take any one ECG vector from the matrix.
4. Export this vector to a text file in which the vector values are in hex.
5. Use this file in the test bench.

Code: 
a = load("name.mat");
a = cell2mat(struct2cell(a));
d = a(1,1:end);
d = d + 200;
z = dec2hex(d);
dlmwrite('input_file.txt',z,'delimiter','');


I. Steps to convert the output file from code to ".mat" file:
1. Import the file using "textscan();".
2. Remove the first 192 bits as they are invalid.
3. Horizontally concatenate the vectors.
4. Convert them to a vector having values in decimal form.

Code: 
filename = 'out.txt';
FID = fopen(filename);
dataFromfile1 = textscan(FID, '%s');% %s for reading string values (hexadecimal numbers)
dataFromfile1 = dataFromfile1{1};
abc = eraseBetween(dataFromfile1{1},1,48);
dataFromfile1{1} = abc;
j = dataFromfile1';
comb1 = j{1};

for i = 2:141
  comb1=horzcat(comb1,j{i});
end

comb3=zeros(6000,1);
k=1;
comb3=cell(6000,1);

for i = 1:3:18000
  comb3{k} = comb1(i:i+2);
  k=k+1;
end
ot = hex2dec(comb3)-200;
plot(ot)
fclose(FID);