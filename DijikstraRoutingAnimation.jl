#clearconsole();
using LightGraphs
using GraphPlot
using MetaGraphs
using GraphRecipes#
using Colors
using ColorSchemes
using Plots
#gr()
#Plots.GRBackend()
pyplot()
#plotly()
# metaGrapher
function metaGrapher(mat)
    mg = MetaDiGraph(size(mat,1))
    set_props!.([mg],vertices(mg),[Dict(:marque => false, :parent=>-1, :distSource => Inf)])
    for i in 1:size(mat,1), j in 1:size(mat,2)
        if mat[i,j] != 0
                add_edge!(mg,i,j,Dict(:weight => mat[i,j],:flow => -1))
        end
    end
    return mg
end
# afficheGraph
function afficheGraph(Mat)
    g = metaGrapher(Mat)
    afficheGraph(g)
end

# returns a random graph adjacency matrix with n vertex
function randomGraphAdj(n)
    mat = rand(1:15*n,n,n)
    for i in 1:n
        mat[i,i] = 0
    end
    return mat
end

# pathfinder
function pathFinder(g,v)
   # imagine that 1 is the source and that we have the following parents
    N=size(g)[1]
    #path= Array{Int}(undef,N)
    path= Array{Int}(undef,N)
    global i=1;
    path[1]=v;

     while(path[i] != -1 )
        path[i+1]=get_prop(g,path[i],:parent)
        i+=1;
     end
     #path=[1,2,3,8];
     if(i<=2)
        pathRet=[];
     else
        pathRet=path[i-1:-1:1];
     end
     return pathRet;
end
##
#Test for PathFinder
g = smallgraph(:petersen)
g = metaGrapher(adjacency_matrix(g))
# imagine that 1 is the source and that we have the following parents
set_prop!(g,8,:parent,3)
set_prop!(g,3,:parent,2)
set_prop!(g,2,:parent,1)
# following the parents we have 1-> 2-> 3-> 8, so we should have [1,2,3,8]
@assert pathFinder(g,8) == [1,2,3,8]
# if the vertex is the source, its path should be empty
@assert pathFinder(g,1) == []
## dijkstra
function dijkstra(Mat,s::Int)
    g1=Mat;
    g = metaGrapher(Mat)
    N=size(g)[1]
    V=vertices(g)
    # We initialize the source with a distance of     0 to the source
    set_prop!(g,s,:distSource,0.0)
    #up=(zeros(1,10))
    for cout=1:N
            # Pick the minimum distance vertex from
            # the set of vertices not yet processed.
            # u is always equal to src in first iteration
            u = minDistance((get_prop.([g],1:N,:distSource)),  get_prop.([g],1:N,:marque))
            println("minDist-->",u)
            set_prop!(g,u,:marque,true)
            for v = 1:N
                if (g1[u,v] > 0.0 && get_prop(g,v,:marque)==false && get_prop(g,v,:distSource) > Float64(get_prop(g,u,:distSource) + g1[u,v]))
                    set_prop!(g,v,:distSource,Float64((get_prop(g,u,:distSource) + g1[u,v])))
                    println(Float64(get_prop(g,u,:distSource) + g1[u,v]))
                end
            end
      end
       #println(pathFinder(g,3))
       nei=[];
       resultH=Vector{Vector{Int64}}([])
       for ip=1:N
           zk=-1;
            ip1=-1;
           while zk!=s
              if(zk==-1)
               ip1=ip;
               set_prop!.([g],1:N,:parent,-1)
               nei=LightGraphs.outneighbors(g,ip1)
               println("neigbour1-->",nei)
              end
              val1=findmin(get_prop.([g],nei,:distSource))
              println(get_prop.([g],nei,:distSource))
              println(val1)
              set_prop!(g,ip1,:parent,nei[val1[2]])
              zk=nei[val1[2]]
              ip1=zk;
              nei=LightGraphs.outneighbors(g,zk)
              println("neigbour2-->",nei)
           end
            if(s!=ip)
                Result=pathFinder(g,ip)
                println(Result)
            else
                Result=[];
                println(Result)
            end

            push!(resultH,Result)
            resultH
       end
          print(resultH)
          return resultH
end

function minDistance(dist,sptSet)
    # Initialize min value
    N=size(dist,1)
    min = Inf;
    min_index= undef;
    for v=1:N
        if (sptSet[v] == false && dist[v] < min)
            min = dist[v];
            min_index = v;
        end
    end
    println(min_index)
    return min_index;
end

#Test Dijkistra (1)
# Petersen graph
g = [ 0  1  0  0  1  1  0  0  0  0
     1  0  1  0  0  0  1  0  0  0
     0  1  0  1  0  0  0  1  0  0
     0  0  1  0  1  0  0  0  1  0
     1  0  0  1  0  0  0  0  0  1
     1  0  0  0  0  0  0  1  1  0
     0  1  0  0  0  0  0  0  1  1
     0  0  1  0  0  1  0  0  0  1
     0  0  0  1  0  1  1  0  0  0
     0  0  0  0  1  0  1  1  0  0]
## to visualize this graph do displayGraph (g)
@assert dijkstra(g,3) == [[3, 2, 1],[3, 2], [], [3, 4],
 [3, 4, 5], [3, 8, 6], [3, 2, 7],
 [3, 8], [3, 4, 9], [3, 8, 10]]
 ## Visualize the GraphPlot
x=[2, 8, 6, 16, 10, 9, 4, 8, 15, 14];
y=[4, 6, 4, 8, 8, 6, 10, 6, 12, 7];
 nodelabel=1:10
 n =10
 edgelabel_dict = Dict()
 edgelabel_dict2 = Dict()
 cs1 =ColorScheme(range(colorant"red", colorant"green", length=100))
 edgelabel_mat = Array{String}(undef, n, n)
 for i in 1:n
     for j in 1:n
         edgelabel_mat[i, j] = edgelabel_dict[(i, j)] = string("edge ", i, " to ", j)
         edgelabel_dict2[(i, j)]=cs1[i];
     end
 end
 GraphRecipes.graphplot(g)
#GraphRecipes.graphplot(g,x=x,y=y,nodesize=3,edgecolor = edgelabel_dict2, names=nodelabel,curvature_scalar=0.1,arrow=arrow(:closed, :head, 1, 1), shorten=0.023)
# Assign x,y to Graph(V,E)
ac=1;
x1=x;
y1=y;
dest=1;
 anim= @animate for i1=1:60
     global x,y,dest
     if(mod(i1,10)==0)
         dest+=1
         ac=1
     end
     ss=dijkstra(g,dest);
     global ac
     #ac=i1;
     for i in 1:n
         for j2 in 1:n
             edgelabel_mat[i, j2] = edgelabel_dict[(i, j2)] = string("edge ", i, " to ", j2);
             edgelabel_dict2[(i, j2)]=nothing;
         end
     end
 if(!isempty(ss[ac]))
     #nodecolor[dest]=colorant"red"
     #nodecolor[ss[ac][end]]=colorant"green"
     spz=reverse(ss[ac]);
     global ab=1;
     while((ab+1)<=size(spz)[1])
         print((spz[ab],spz[ab+1]))
         sp1=[spz[ab], spz[ab+1]]
         #edgelabel_dict2[(spz[ab],spz[ab+1])]=:red;
         edgelabel_dict2[(sort(sp1)[1],sort(sp1)[2])]=:red;
         ab+=1;
     end

     membership = [1,1,1,1,1,1,1,1,1,1]
     nodecolor2 = range(colorant"lightyellow", stop=colorant"yellow", length=n)
     nodecolor2[dest]=colorant"red"
     nodecolor2[spz[1]]=colorant"green"

     nodeshape2=[:hexagon]
     nodefillc1 = nodecolor2#[membership]
     nodeshape1 = nodeshape2[membership]

     node_weights = 1:n
    # nodecolor = range(colorant"yellow", stop=colorant"red", length=n)

     x=x.+rand(10)
     y=y.+rand(10)
    # print(x)
     (GraphRecipes.graphplot(g,x=x,y=y,linewidth=1, self_edge_size=0.25,  nodestrokewidth=1.5,
              markerstrokestyle=:solid,
              markerstrokealpha=1,
              markerstrokecolor=:black,markersize=2,nodeshape=nodeshape1,fontsize=10,nodesize=1,edgecolor = edgelabel_dict2,nodecolor=nodefillc1, names=nodelabel,curvature_scalar=0.08,arrow=arrow(:closed, :head,1, 1), shorten=0.003));
              title!("Dijkstra Routing with Animation(JULIA)")
 end
     ac+=1;
end
gif(anim, "anim_fps15.gif", fps = 5)
mp4(anim, "movie.mp4", fps=4)
