# This file contains methods to generate a data set of instances 
include("io.jl")

"""
Generate an n*n grid 

Argument
- n: size of the grid
"""
function generateInstance(n::Int64)
    if n<=3
        println("size should be greater than 3")
        return
    end
	b=n # number of black cells
    directions=[1 0;-1 0;0 1;0 -1;1 1;1 -1;-1 -1;-1 1]
    grid=trues(n,n)
    t=zeros(Int,n,n)
    # put black cells and keep the white cells continuous
    for r=1:n
        canPutBlackCell=false
        while !canPutBlackCell
            c=rand(1:n)
            canPutBlackCell=true
            for i=1:size(directions,1)
                if r+directions[i,1]>=1 && r+directions[i,1]<=n && c+directions[i,2]>=1 && c+directions[i,2]<=n && grid[r+directions[i,1],c+directions[i,2]]==0
                    canPutBlackCell=false
                end
            end
            if canPutBlackCell==true
                grid[r,c]=0
            end
        end
    end
    for r=1:n
        canPutNumberCell=false
        while !canPutNumberCell
            c=rand(1:n)
            if grid[r,c]!=0
                visNum=visiable(grid,r,c)
                t[r,c]=visNum
                canPutNumberCell=true
            end
        end
    end
    # println(grid)
    return t
end 
"""
count the visiable cells from position (l,c)
"""
function visiable(x_vals::BitArray{2},l::Int,c::Int)
    rowNum=size(x_vals,1)
    colNum=size(x_vals,2)

    up=0 # the number of white cells in top direction until meet black cell
    right=0 # the number of white cells in right direction until meet black cell
    down=0 # the number of white cells in down direction until meet black cell
    left=0 # the number of white cells in left direction until meet black cell
    for i1 in 1:l-1
        if  x_vals[l-i1,c]== 0  #meet the black cell
            down=i1-1
            break
        else
            down=l-1
        end
    end
    for j1 in 1:rowNum-l
        if  x_vals[l+j1,c]== 0
            up=j1-1
            break
        else
            up=rowNum-l
        end
    end
    for i2 in 1:c-1
        if  x_vals[l,c-i2]== 0
            left=i2-1
            break
        else
            left=c-1
        end
    end
    for j2 in 1:colNum-c
        if  x_vals[l,c+j2]== 0
            right=j2-1
            break
        else
            right=colNum-c
        end
    end
    return up+down+left+right+1
end
"""
Generate all the instances

Remark: a grid is generated only if the corresponding output file does not already exist
"""
function generateDataSet()
	# different grid size
	for n in [4,5,6]
		for m in [4,5,6]
			#generate 2 instances for each size
			for instance=1:10
				fileName="../data/instance_$(n)_$(m)_$(instance).txt"
				if !isfile(fileName)
					println("-- Generating file " * fileName)
	                saveInstance(generateInstance(n), fileName)
	            end
	        end
	    end
    end    
end



