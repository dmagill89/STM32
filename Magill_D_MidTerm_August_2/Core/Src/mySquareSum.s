.text
.global SquareSum

SquareSum:
 MOV r1, #0
 MOV r2, #0

loop:
 CMP r2, r0
 BGT endloop
 MLA r1, r2, r2, r1
 ADD r2, #1
 B loop
endloop:

 MOV r0, r1
 MOV pc, lr
