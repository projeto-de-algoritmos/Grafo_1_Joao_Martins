
from collections import defaultdict

class Graph():

    def __init__(self, graph_dict=None):
        """ initializes a graph object 
            If no dictionary or None is given, an empty dictionary will be used
        """
        if graph_dict == None:
            graph_dict = {}
        self.__graph_dict = graph_dict


    def vertices(self):
        """ returns the vertices of a graph """
        return list(self.__graph_dict.keys())

    def edges(self):
        """ returns the edges of a graph """
        return self.__generate_edges()

    def add_vertex(self, vertex):
        """ If the vertex "vertex" is not in 
            self.__graph_dict, a key "vertex" with an empty
            list as a value is added to the dictionary. 
            Otherwise nothing has to be done. 
        """
        if vertex not in self.__graph_dict:
            self.__graph_dict[vertex] = []

    def add_edge(self, edge):
        """ assumes that edge is of type set, tuple or list; 
            between two vertices can be multiple edges! 
        """
        edge = set(edge)
        vertex1 = edge.pop()
        if edge:
            # not a loop
            vertex2 = edge.pop()
        else:
            # a loop
            vertex2 = vertex1
        if vertex1 in self.__graph_dict:
            self.__graph_dict[vertex1].append(vertex2)
        else:
            self.__graph_dict[vertex1] = [vertex2]

    def __generate_edges(self):
        """ A static method generating the edges of the 
            graph "graph". Edges are represented as sets 
            with one (a loop back to the vertex) or two 
            vertices 
        """
        edges = []
        for vertex in self.__graph_dict:
            for neighbour in self.__graph_dict[vertex]:
                if {neighbour, vertex} not in edges:
                    edges.append({vertex, neighbour})
        return edges

    def __str__(self):
        res = "vertices: "
        for k in self.__graph_dict:
            res += str(k) + " "
        res += "\narestas: "
        for edge in self.__generate_edges():
            res += str(edge) + " "
        return res 

    def find_isolated_vertices(self):
        """ returns a list of isolated vertices. """
        graph = self.__graph_dict
        isolated = []
        for vertex in graph:
            print(isolated, vertex)
            if not graph[vertex]:
                isolated += [vertex]
        return isolated

    def find_path(self, start_vertex, end_vertex, path=[]):
        """ find a path from start_vertex to end_vertex 
            in graph """
        graph = self.__graph_dict
        path = path + [start_vertex]
        if start_vertex == end_vertex:
            return path
        if start_vertex not in graph:
            return None
        for vertex in graph[start_vertex]:
            if vertex not in path:
                extended_path = self.find_path(vertex, 
                                               end_vertex, 
                                               path)
                if extended_path: 
                    return extended_path
        return None
    

    def find_all_paths(self, start_vertex, end_vertex, path=[]):
        """ find all paths from start_vertex to 
            end_vertex in graph """
        graph = self.__graph_dict 
        path = path + [start_vertex]
        if start_vertex == end_vertex:
            return [path]
        if start_vertex not in graph:
            return []
        paths = []
        for vertex in graph[start_vertex]:
            if vertex not in path:
                extended_paths = self.find_all_paths(vertex, 
                                                     end_vertex, 
                                                     path)
                for p in extended_paths: 
                    paths.append(p)
        return paths

    def is_connected(self, 
                     vertices_encountered = None, 
                     start_vertex=None):
        """ Verifica se o grafo é conectado"""
        if vertices_encountered is None:
            vertices_encountered = set()
        gdict = self.__graph_dict        
        vertices = list(gdict.keys())
        if not start_vertex:
            start_vertex = vertices[0]
        vertices_encountered.add(start_vertex)
        if len(vertices_encountered) != len(vertices):
            for vertex in gdict[start_vertex]:
                if vertex not in vertices_encountered:
                    if self.is_connected(vertices_encountered, vertex):
                        return True
        else:
            return True
        return False

    def vertex_degree(self, vertex):
        """ Grau de saída do vértice""" 
        adj_vertices =  self.__graph_dict[vertex]
        degree = len(adj_vertices) + adj_vertices.count(vertex)
        return degree

    def reverse(self):
        """Gera o grafo reverso"""
        r = {key: [] for key in self.__graph_dict}
        for key, value in self.__graph_dict.items():
            for values in value:
                r[values].append(key)
        return dict(r)

    # def degree_sequence(self):
    #     """ calculates the degree sequence """
    #     seq = []
    #     for vertex in self.__graph_dict:
    #         seq.append(self.vertex_degree(vertex))
    #     seq.sort(reverse=True)
    #     return tuple(seq)
  

    def min_degree(self):
        """ the minimum degree of the vertices """
        min = 100000000
        for vertex in self.__graph_dict:
            vertex_degree = self.vertex_degree(vertex)
            if vertex_degree < min:
                min = vertex_degree
        return min
        
    def max_degree(self):
        """ the maximum degree of the vertices """
        max = 0
        for vertex in self.__graph_dict:
            vertex_degree = self.vertex_degree(vertex)
            if vertex_degree > max:
                max = vertex_degree
        return max

    def BFS(self, start):
        # visited = [False] * (len(self.__graph_dict)) 
        # queue = [] 
        # queue.append(s) 
        # visited[s-1] = True
  
        # while queue: 
        #     s = queue.pop(0) 
        #     print(s, end = " ")

        #     for i in self.__graph_dict[s]:
        #         if visited[i-1] == False: 
        #             queue.append(i) 
        #             visited[i-1] = True
        visited, queue = set(), [start]
        while queue:
            vertex = queue.pop(0)
            if vertex not in visited:
                visited.add(vertex)
                queue.extend(self.__graph_dict[vertex])
        return visited

    def DFS(self, s, visited):
        if s not in visited:
            visited.append(s)
            for n in self.__graph_dict[s]:
                self.DFS(n, visited)
        return visited

if __name__ == '__main__':

    # g = { "a" : ["c"],
    #       "b" : ["c", "e"],
    #       "c" : ["a", "b", "d", "e"],
    #       "d" : ["c"],
    #       "e" : ["c", "b"],
    #       "f" : []
    #     }
    g2 = {
        1: [2, 3, 8, 9, 10, 11, 12], #tartaruga marinha
        2: [12], #polvo
        3: [5, 10, 11],#peixe
        4: [1],#esponja
        5: [1],#agua viva
        6: [1, 7], #alga
        7: [1],#crustaceos
        8: [],#cachorro
        9: [],#raposa
        10: [],#focas
        11: [],#gaivotas
        12: [10] #tubarao
    }

    graph = Graph(g2)
    print("Grafo inicial: ")
    print(g2)
    print("Grau de cada vértice no grafo inicial")
    for node in graph.vertices():
        print("Vértice ", node, ": -> ", graph.vertex_degree(node))
    for node in graph.vertices():
        print("BFS do grafo inicial partindo do vértice ", node, ":")
        print(graph.BFS(node), "\n")

    for node in graph.vertices():
        print("DFS do grafo inicial partindo do vértice ", node, ":")
        print(graph.DFS(node, []), "\n")

    print("\nGrafo reverso:")
    graph_rev = graph.reverse()
    print(graph_rev)
    reversed_g = Graph(graph_dict=graph_rev)
    

    print("\nGrau de cada vértice no grafo reverso")
    for node in reversed_g.vertices():
        print("Vértice ", node, ": -> ", reversed_g.vertex_degree(node))

    for node in reversed_g.vertices():
        print("BFS do grafo reverso partindo do vértice ", node, ":")
        print(reversed_g.BFS(node), "\n")

    for node in reversed_g.vertices():
        print("DFS do grafo reverso partindo do vértice ", node, ":")
        print(reversed_g.DFS(node, []), "\n")

    print("O maior grau de um vértice no grafo (topo de cadeia):", graph.max_degree())
 
    print("O maior grau de um vértice no grafo reverso:", reversed_g.max_degree())

    print("O menor grau de um vértice no grafo (produtores):", graph.min_degree())

    print("O menor grau de um vértice no grafo reverso:", reversed_g.min_degree())

    print("DFS no grafo reverso:")

    print(reversed_g.DFS(1, []))