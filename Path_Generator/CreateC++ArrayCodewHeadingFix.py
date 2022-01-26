#int x[3][4] = {{0,1,2,3}, {4,5,6,7}, {8,9,10,11}};
import csv
import os

inputfile = "straight"
with open(inputfile+'_left.csv', 'rU') as csvfile:

    csvstr = str(csvfile.name)
    
    f= open(csvstr[:-4]+".txt","w+")
    
    readCSV = csv.reader(csvfile, delimiter=',')
    row_count = sum(1 for row in readCSV)

    csvfile.seek(0)
    next(readCSV)

    f.write("double "+ csvstr[:-4]+" ["+str(row_count-1)+"] [4] = {")
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

        pos = float(row[1])*6767/5
        velo = float(row[2])*6767/5
        if row_count-2 == ct:
            f.write("{"+row[0]+","+str(pos)+","+str(velo)+","+str(-H)+"}};")
        else:
            f.write("{"+row[0]+","+str(pos)+","+str(velo)+","+str(-H)+"},")
        #print(row[0],row[1],row[2],)
        print(str(-H))
f.close() 
with open(inputfile+'_right.csv', 'rU') as csvfile:

    csvstr = str(csvfile.name)
    
    f= open(csvstr[:-4]+".txt","w+")
    
    readCSV = csv.reader(csvfile, delimiter=',')
    row_count = sum(1 for row in readCSV)

    csvfile.seek(0)
    next(readCSV)

    f.write("double "+ csvstr[:-4]+" ["+str(row_count)+"] [4] = {")
    
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

        pos = float(row[1])*6767/5
        velo = float(row[2])*6767/5
        if row_count-2 == ct:
            f.write("{"+row[0]+","+str(pos)+","+str(velo)+","+str(-H)+"}};")
        else:
            f.write("{"+row[0]+","+str(pos)+","+str(velo)+","+str(-H)+"},")
        #print(row[0],row[1],row[2],)
        print(str(-H))

f.close() 
