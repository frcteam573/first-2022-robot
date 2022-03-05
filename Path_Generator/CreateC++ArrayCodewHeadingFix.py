#int x[3][4] = {{0,1,2,3}, {4,5,6,7}, {8,9,10,11}};
import csv
import os

inputfile = "4BallPath1"
reverse = False

with open(inputfile+'_left.csv', 'rU') as csvfile:

    csvstr = str(csvfile.name)

    if reverse:
        f = open(csvstr[:-8]+"right.txt","w+")
        vari = csvstr[:-8]+"right"
    else:
        f= open(csvstr[:-4]+".txt","w+")
        vari = csvstr[:-4]
    
    readCSV = csv.reader(csvfile, delimiter=',')
    row_count = sum(1 for row in readCSV)

    csvfile.seek(0)
    next(readCSV)

    f.write("double "+ vari+" ["+str(row_count-1)+"] [4] = {")
    h_old = 0
    h_init = 0
    for ct, row in enumerate(readCSV):
        #Heading Conversion#
        raw_heading = float(row[3])
        if ct == 0:
            h_init = raw_heading
        H = (raw_heading-h_init)*180/3.145359


        if abs(H-h_old) > 10:
            if h_old > 360:
                H = H+360
            elif h_old < -360:
                H = H-720
            else:
                H = H -360

        h_old = H
    
        
        
        ######

        # 

        pos = float(row[1])*200.6 # ratio from ft to ticks
        velo = float(row[2])*200.6 # ratio from ft to ticks
        if reverse:
            pos = -1*pos
            velo = -1*velo
        if row_count-2 == ct:
            f.write("{"+row[0]+","+str(pos)+","+str(velo)+","+str(-H)+"}};")
        else:
            f.write("{"+row[0]+","+str(pos)+","+str(velo)+","+str(-H)+"},")
        #print(row[0],row[1],row[2],)
        print(str(-H))
f.close() 
with open(inputfile+'_right.csv', 'rU') as csvfile:

    csvstr = str(csvfile.name)
    
    if reverse:
        f = open(csvstr[:-9]+"left.txt","w+")
        vari = csvstr[:-9]+"left"
    else:
        f= open(csvstr[:-4]+".txt","w+")
        vari = csvstr[:-4]
    
    readCSV = csv.reader(csvfile, delimiter=',')
    row_count = sum(1 for row in readCSV)

    csvfile.seek(0)
    next(readCSV)

    f.write("double "+ vari+" ["+str(row_count)+"] [4] = {")
    
    h_old = 0
    for ct, row in enumerate(readCSV):
        #Heading Conversion#
        raw_heading = float(row[3])
        if ct == 0:
            h_init = raw_heading
        H = (raw_heading-h_init)*180/3.145359


        if abs(H-h_old) > 10:
            if h_old > 360:
                H = H+360
            elif h_old < -360:
                H = H-720
            else:
                H = H -360

        h_old = H
    
        
        
        ######

        pos = float(row[1])*200.6
        velo = float(row[2])*200.6
        
        if reverse:
            pos = -1*pos
            velo = -1*velo
            
        if row_count-2 == ct:
            f.write("{"+row[0]+","+str(pos)+","+str(velo)+","+str(-H)+"}};")
        else:
            f.write("{"+row[0]+","+str(pos)+","+str(velo)+","+str(-H)+"},")
        #print(row[0],row[1],row[2],)
        #print(str(-H))

f.close() 
