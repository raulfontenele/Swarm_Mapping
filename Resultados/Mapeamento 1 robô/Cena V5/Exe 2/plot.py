# -*- coding: utf-8 -*-
"""
Created on Wed Mar 24 21:24:36 2021

@author: raulf
"""

#import plotly

import plotly.graph_objects as go
import json
import matplotlib.pyplot as plt

def fun1():
    edge_x = []
    edge_y = []
    '''
    for edge in G.edges():
        x0, y0 = G.nodes[edge[0]]['pos']
        x1, y1 = G.nodes[edge[1]]['pos']
        edge_x.append(x0)
        edge_x.append(x1)
        edge_x.append(None)
        edge_y.append(y0)
        edge_y.append(y1)
        edge_y.append(None)
    '''

    file = open('novomapa.txt','r')
    lines = file.readlines()

    for line in lines:
        line = line.replace("'",'"')
        struct =json.loads(line)
        for neighbor in struct["neighborhood"]:
            edge_x.append(struct["nodeCoord"][0])
            edge_x.append(struct["nodeCoord"][1])
            edge_x.append(None)
            edge_y.append(neighbor[0])
            edge_y.append(neighbor[1])
            edge_y.append(None)


    edge_trace = go.Scatter(
        x=edge_x, y=edge_y,
        line=dict(width=0.5, color='#888'),
        hoverinfo='none',
        mode='lines')

    node_x = []
    node_y = []
    for line in lines:
        line = line.replace("'",'"')
        struct =json.loads(line)

        node_x.append(struct["nodeCoord"][0])
        node_y.append(struct["nodeCoord"][1])

    node_trace = go.Scatter(
        x=node_x, y=node_y,
        mode='markers',
        hoverinfo='text',
        marker=dict(
            showscale=True,
            # colorscale options
            #'Greys' | 'YlGnBu' | 'Greens' | 'YlOrRd' | 'Bluered' | 'RdBu' |
            #'Reds' | 'Blues' | 'Picnic' | 'Rainbow' | 'Portland' | 'Jet' |
            #'Hot' | 'Blackbody' | 'Earth' | 'Electric' | 'Viridis' |
            colorscale='YlGnBu',
            reversescale=True,
            color=[],
            size=10,
            colorbar=dict(
                thickness=15,
                title='Node Connections',
                xanchor='left',
                titleside='right'
            ),
            line_width=2))

    fig = go.Figure(data=[edge_trace, node_trace],
                layout=go.Layout(
                    title='<br>Network graph made with Python',
                    titlefont_size=16,
                    showlegend=False,
                    hovermode='closest',
                    margin=dict(b=20,l=5,r=5,t=40),
                    annotations=[ dict(
                        text="Python code: <a href='https://plotly.com/ipython-notebooks/network-graphs/'> https://plotly.com/ipython-notebooks/network-graphs/</a>",
                        showarrow=False,
                        xref="paper", yref="paper",
                        x=0.005, y=-0.002 ) ],
                    xaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
                    yaxis=dict(showgrid=False, zeroline=False, showticklabels=False))
                    )
    fig.show()

def fun2():
    file = open('map.txt','r')
    lines = file.readlines()

    for line in lines:
        line = line.replace("'",'"')
        struct =json.loads(line)
        for neighbor in struct["neighborhood"]:
            plt.plot( [struct["nodeCoord"][0],neighbor[0]],[struct["nodeCoord"][1],neighbor[1]])
            plt.scatter(struct["nodeCoord"][0],struct["nodeCoord"][1],s = 1000,marker='h')
    
    plt.show()

fun2()