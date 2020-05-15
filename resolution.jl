# This file contains methods to solve an instance (heuristically or with CPLEX)
using CPLEX
using Printf
include("generation.jl")

TOL = 0.00001
directions=[1 0;-1 0;0 1;0 -1]

"""
Solve an instance with CPL

Argument
-t: array which stores the puzzle data

Return

"""


function cplexSolve(t::Array{Int,2})

    rowNum=size(t,1)
    colNum=size(t,2)
    if rowNum!=colNum
        println("not square matrix")
        return
    end
    @printf("%d-%d\n",rowNum,colNum)
    # Create the model
    m = Model(CPLEX.Optimizer)

    # x[i,j]=1 if the cell is white, 0 if the cell is black
    @variable(m,x[1:rowNum,1:colNum],Bin)

    ss=0
    # set the cell white if it has a number
    for l in 1:rowNum
        for c in 1:colNum
            if t[l,c]!=0
                @constraint(m,x[l,c]==1)
                ss=ss+1
            end
        end
    end
    
    # No two black squares are orthogonally adjacent
    @constraint(m,[r in 1:rowNum-1, c in 1:colNum],x[r,c]+x[r+1,c]>=1)
    @constraint(m,[r in 1:rowNum, c in 1:colNum-1],x[r,c]+x[r,c+1]>=1)

    # For any two white squares, there is a path between them using only white squares.
    @constraint(m,[c in 2:colNum-1],sum(x[1,c-1:c+1])+x[2,c]>=2) # the first line except the corners
    @constraint(m,[c in 2:colNum-1],sum(x[rowNum,c-1:c+1])+x[rowNum-1,c]>=2) # the last line except the corners
    @constraint(m,[r in 2:rowNum-1],sum(x[r-1:r+1,1])+x[r,2]>=2) # the first column except the corners
    @constraint(m,[r in 2:rowNum-1],sum(x[r-1:r+1,colNum])+x[r,colNum-1]>=2) # the last colNum except the corners
    @constraint(m,sum(x[1:2,1])+x[1,2]>=2) # the left top corner
    @constraint(m,sum(x[1:2,colNum])+x[1,colNum-1]>=2) # the right top corner
    @constraint(m,sum(x[rowNum-1:rowNum,1])+x[rowNum,2]>=2) # the left bottom corner
    @constraint(m,sum(x[rowNum-1:rowNum,colNum])+x[rowNum,colNum-1]>=2) # the right bottom corner

    @constraint(m,[r in 2:rowNum-1, c in 2:colNum-1],sum(x[r-1:r+1,c-1:c+1])>=2)

    #case when the black is in diagonal line
    @constraint(m,sum(x[r,r] for r=1:rowNum)>=1)
    @constraint(m, sum(x[rowNum-r+1,r] for r=1:rowNum)>=1)

    function my_callback_function(cb_data)
        x_vals = callback_value.(Ref(cb_data), x)
        zz=0
        para=Array{Int64,2}(undef,0,6)
        for l in 1:rowNum
            for c in 1:colNum
                if t[l,c]!=0
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
                    if up+down+left+right+1>t[l,c]
                        # println(up+down+left+right+1)                   
                        con1 = @build_constraint((sum(x[l:l+up,c])+sum(x[l-down:l,c])+sum(x[l,c-left:c])+sum(x[l,c:c+right])-3)>=t[l,c]) #counting the white cells from four orthogonal directions
                        # # # con1=@build_constraint(up+down+left+right+1==t[l,c])
                        MOI.submit(m, MOI.LazyConstraint(cb_data), con1)
                    elseif up+down+left+right+1<t[l,c]
                        con = @build_constraint((sum(x[l:l+up,c])+sum(x[l-down:l,c])+sum(x[l,c-left:c])+sum(x[l,c:c+right])-3)<=t[l,c])
                        MOI.submit(m, MOI.LazyConstraint(cb_data), con)
                    elseif up+down+left+right+1==t[l,c]
                        zz=zz+1
                        para=vcat(para,[l c up down left right])
                    end                    
                end
            end
        end
        # println(zz,"/",ss)
        if(zz>=ss)
            
            for i=1:ss
                l,c,up,down,left,right=para[i,:]
                # @printf("func:%d+%d+%d+%d+1=%d\n",up,down,left,right,t[l,c]) #up,down,left,right,target
                # con = @build_constraint((sum(x[l:l+up,c])+sum(x[l-down:l,c])+sum(x[l,c-left:c])+sum(x[l,c:c+right])-3)==t[l,c])
                
                # MOI.submit(m, MOI.LazyConstraint(cb_data), con)
            
            end
        end
    end

    MOI.set(m, MOI.LazyConstraintCallback(), my_callback_function)
    # Maximize the number of white cells
    @objective(m,Max, sum(x))

    # Start a chronometer
    start = time()

    # Solve the model
    optimize!(m)

    # println(num_constraints(m, VariableRef, MOI.ZeroOne))
    # println("-----",termination_status(m))
    # Return:
    # 1 - true if an optimum is found(type: Bool)
    # 2 - the resolution time(type Float64)
    # 3 - the value of each cell(type:Array{undef,2})
    return JuMP.primal_status(m) == JuMP.MathOptInterface.FEASIBLE_POINT, time() - start,x
    
end

"""
Heuristically solve an instance
"""
function visiable(x_vals::Array{Bool, 2},l::Int,c::Int)
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
    return up+down+left+right+x_vals[l,c]
end

function dfs(l::Int,c::Int,x_vals::Array{Bool, 2},walked::Array{Bool, 2})
    
    rowNum=size(x_vals,1)
    colNum=size(x_vals,2)
    ans=1
    for i in 1:4
        r1=l+directions[i,1]
        c1=c+directions[i,2]
        if r1>=1 && r1<=rowNum && c1>=1 && c1<=colNum && walked[r1,c1]==0 && x_vals[r1,c1]!=0
            walked[r1,c1]=1
            ans+=dfs(r1,c1,x_vals,walked)
        end
    end
    walked=zeros(Bool,rowNum,colNum)
    return ans
end

"""
check if it is possible to place a black cell in position(r,c) and if possible, place black cell
return true if possible, false if impossible
"""
function placeBlack(r::Int, c::Int,x_vals::Array{Bool,2})
    rowNum=size(x_vals,1)
    colNum=size(x_vals,2)
    # if the cell is already a black one
    if x_vals[r,c]!=1
        return false
    end

    # there is black cells in its 4 orthogonal neighbors 
    for i=1:size(directions,1)
        if r+directions[i,1]>=1 && r+directions[i,1]<=rowNum && c+directions[i,2]>=1 &&c+directions[i,2]<=colNum && x_vals[r+directions[i,1],c+directions[i,2]]==0
            return false
        end
    end
    x_vals[r,c]=0
    return true
end

"""
unplace the black cell, change it to white
"""
function unplaceBlack(r::Int, c::Int,x_vals::Array{Bool,2})
    x_vals[r,c]=1
end

"""
iterativly solve the puzzle
"""
function solve(r::Int,c::Int,rw::Int,cw::Int,x_vals::Array{Bool,2},walked::Array{Bool,2},t::Array{Int,2})
    rowNum=size(x_vals,1)
    colNum=size(x_vals,2)

    if c > colNum
        c = 1
        r+=1
    end
    # check if white cells are continuous
    reach=0
    walked[rw,cw]=1
    if x_vals[rw,cw]!=1
        x_vals[rw,cw]=1
        reach+=dfs(rw,cw,x_vals,walked)
        x_vals=0
    else
        reach+=dfs(rw,cw,x_vals,walked)
    end
    walked=zeros(Bool,rowNum,colNum)
    if reach!=sum(x_vals)
        # println("white cells are not contiguous,reach=",reach,"sum=",sum(x_vals))
        return 0
    end
    # check after all iteration whether the puzzle is finally solve
    if r>rowNum
        solved=true
        for i in 1:rowNum
            for j in 1:colNum
                if t[i,j]>0 # for the cell who has a number
                    if visiable(x_vals,i,j)!=t[i,j] # not solved
                        # println("no solution")
                        solved=false
                        return 0
                    end
                end
            end
        end
        if solved==true
            # finally solved
            # println("solve successfully")
            # println(x_vals)
            return x_vals
        end
        return 0
    end
    # check in each iteration
    for i in 1:rowNum
        for j in 1:colNum
            if t[i,j]>0 # for the cell who has a number
                if visiable(x_vals,i,j)<t[i,j] # not solved
                    # println(visiable(x_vals,i,j)," less than ",t[i,j])
                    return 0
                end
                if i<r && visiable(x_vals[1:r-1,:],i,j)>t[i,j]
                    # println("before ",r)
                    return 0
                end
                # println("nothing:",visiable(x_vals,i,j),"->",t[i,j])
            end
        end
    end
    if placeBlack(r,c,x_vals)==true
        rw1=rw
        cw1=cw
        # update the position of white cell
        if rw1==r && cw1==c
            cw1+=1 # move right
            if cw1>colNum
                cw1=1
                rw1+=1
            end
        end
        # println("new->",r,c+1,rw1,cw1,x_vals,walked,t)
        ret_val=solve(r,c+1,rw1,cw1,x_vals,walked,t)
        # println(ret_val)
        if ret_val!=0 && ret_val!=nothing
            return ret_val
        end
        unplaceBlack(r,c,x_vals)
    end
    walked=zeros(Bool,rowNum,colNum)
    # println("new start",r,c+1,rw,cw,x_vals,walked,t)
    ret_val=solve(r,c+1,rw,cw,x_vals,walked,t)
    if ret_val!=0 && ret_val!=nothing
        return ret_val
    end
    # println(ret_val)
end

function heuristicSolve(t::Array{Int, 2})
    rowNum=size(t,1)
    colNum=size(t,2)
    walked=zeros(Bool,rowNum,colNum)
    x_vals=ones(Bool,rowNum,colNum)
    ret_val=solve(1,1,1,1,x_vals,walked,t)
    isOptimal=false
    if ret_val!=0 && ret_val!=nothing
        isOptimal=true
        println("solve successfully")
        println(ret_val)
    else
        isOptimal=false
        println("no solution")
        println(ret_val)
    end
    return isOptimal,ret_val
end 

"""
Solve all the instances contained in "../data" through CPLEX and heuristics

The results are written in "../res/cplex" and "../res/heuristic"

Remark: If an instance has previously been solved (either by cplex or the heuristic) it will not be solved again
"""
function solveDataSet()

    dataFolder = "../data/"
    resFolder = "../res/"

    # Array which contains the name of the resolution methods
    # resolutionMethod = ["cplex"]
    resolutionMethod = ["cplex", "heuristique"]

    # Array which contains the result folder of each resolution method
    resolutionFolder = resFolder .* resolutionMethod

    # Create each result folder if it does not exist
    for folder in resolutionFolder
        if !isdir(folder)
            mkdir(folder)
        end
    end
            
    global isOptimal = false
    global solveTime = -1

    # For each instance
    # (for each file in folder dataFolder which ends by ".txt")
    for file in filter(x->occursin(".txt", x), readdir(dataFolder))  
        
        println("-- Resolution of ", file)
        t=readInputFile(dataFolder * file)

        # For each resolution method
        for methodId = 1:size(resolutionMethod,1)
            
            outputFile = resolutionFolder[methodId] * "/" * file

            # If the instance has not already been solved by this method
            if !isfile(outputFile)
                
                fout = open(outputFile, "w")  

                resolutionTime = -1
                isOptimal = false
                
                # If the method is cplex
                if resolutionMethod[methodId] == "cplex"
                    
                    # Solve it and get the results
                    isOptimal, resolutionTime,x = cplexSolve(t)
                    
                    # If a solution is found, write it
                    if isOptimal
                        writeSolution(fout, x)
                    end

                # If the method is one of the heuristics
                else
                    
                    isSolved = false

                    # Start a chronometer 
                    startingTime = time()
                    
                    solution=nothing

                    # While the grid is not solved and less than 100 seconds are elapsed
                    
                    # Solve it and get the results
                    isOptimal, solution = heuristicSolve(t)

                    # Stop the chronometer
                    resolutionTime = time() - startingTime

                    # Write the solution (if any)
                    if isOptimal

                        writeSolution(fout, solution)
                        
                    end 
                end

                println(fout, "solveTime = ", resolutionTime) 
                println(fout, "isOptimal = ", isOptimal)
                
                close(fout)
            end


            # Display the results obtained with the method on the current instance
            include(outputFile)
            println(resolutionMethod[methodId], " optimal: ", isOptimal)
            println(resolutionMethod[methodId], " time: " * string(round(solveTime, sigdigits=2)) * "s\n")
        end         
    end 
end
