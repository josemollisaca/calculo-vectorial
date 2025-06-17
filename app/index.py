from dash import Dash, dcc, html, Input, Output, State, callback_context
import plotly.graph_objs as go
import numpy as np
from flask import Flask

# Inicializar Flask y Dash
server = Flask(__name__)
app = Dash(__name__, server=server, suppress_callback_exceptions=True)

# Configuración para Vercel (CRUCIAL)
application = server

# Parámetros iniciales
parametros_iniciales = {
    'a': 1.0,
    'velocidad_maxima': 1.0,
    'factor_suavizado': 0.1,
    'control_adaptativo': True,
    'limite_curvatura': 2.0
}

simulacion_inicial = {
    'esta_corriendo': False,
    'tiempo_actual': 0.0,
    'posicion_robot': {'x': 0, 'y': 0},
    'trayectoria': [],
    'problemas': [],
    'soluciones': []
}

# Funciones matemáticas
def folio_parametrico(t, a=1.0):
    denominador = 1 + t**3
    x = (3 * a * t) / denominador
    y = (3 * a * t**2) / denominador
    return x, y

def calcular_curvatura(t, a=1.0):
    if t < 0.001:
        return 0
    denominador = 1 + t**3
    denominador2 = denominador**2
    
    dx_dt = (3 * a * (1 - 2 * t**3)) / denominador2
    dy_dt = (3 * a * t * (2 - t**3)) / denominador2
    d2x_dt2 = (18 * a * t * (t**3 - 2)) / (denominador2 * denominador)
    d2y_dt2 = (6 * a * (1 - 4 * t**3 + t**6)) / (denominador2 * denominador)
    
    numerador = abs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2)
    denominador = (dx_dt**2 + dy_dt**2)**1.5
    return numerador / denominador if denominador > 0.001 else 0

def calcular_velocidad_adaptativa(curvatura, vel_max, limite_curvatura, control_adaptativo):
    if not control_adaptativo:
        return vel_max
    factor_velocidad = min(1.0, limite_curvatura / (curvatura + 0.1))
    return vel_max * max(0.1, factor_velocidad)

def identificar_problemas(t, curvatura, velocidad, a):
    problemas = []
    x, y = folio_parametrico(t, a)
    posicion = {'x': x, 'y': y}
    distancia_desde_origen = np.sqrt(x**2 + y**2)
    
    if curvatura > 5.0:
        problemas.append({
            'tipo': 'curvatura_alta',
            'severidad': 'crítico',
            'descripcion': f'Curvatura extrema: κ = {curvatura:.2f}',
            'posicion': posicion,
            'recomendacion': 'Reducir velocidad significativamente'
        })
    
    if distancia_desde_origen < 0.1:
        problemas.append({
            'tipo': 'singularidad',
            'severidad': 'crítico',
            'descripcion': 'Cerca de singularidad matemática',
            'posicion': posicion,
            'recomendacion': 'Implementar bypass o parada controlada'
        })
    
    if 0.5 < t < 2.0 and curvatura > 2.0:
        problemas.append({
            'tipo': 'giro_brusco',
            'severidad': 'advertencia',
            'descripcion': 'Cambio brusco de dirección detectado',
            'posicion': posicion,
            'recomendacion': 'Aplicar suavizado de trayectoria'
        })
    
    if velocidad * curvatura > 3.0:
        problemas.append({
            'tipo': 'velocidad_curvatura',
            'severidad': 'advertencia',
            'descripcion': 'Velocidad muy alta para la curvatura local',
            'posicion': posicion,
            'recomendacion': 'Activar control adaptativo'
        })
    
    return problemas

def generar_soluciones(problemas):
    soluciones = []
    for problema in problemas:
        if problema['tipo'] == 'curvatura_alta':
            soluciones.append({
                'titulo': 'Control de Velocidad Adaptativo',
                'descripcion': 'v(κ) = v_max × min(1, κ_limit/κ)',
                'implementacion': 'Sensor de curvatura local + controlador PID',
                'efectividad': 90
            })
        elif problema['tipo'] == 'singularidad':
            soluciones.append({
                'titulo': 'Bypass de Singularidad',
                'descripcion': 'Trayectoria alternativa que evita r < r_min',
                'implementacion': 'Planificador de rutas con restricciones geométricas',
                'efectividad': 95
            })
        elif problema['tipo'] == 'giro_brusco':
            soluciones.append({
                'titulo': 'Suavizado Gaussiano',
                'descripcion': 'Filtro σ(t) sobre la trayectoria original',
                'implementacion': 'Convolución con kernel gaussiano',
                'efectividad': 75
            })
        elif problema['tipo'] == 'velocidad_curvatura':
            soluciones.append({
                'titulo': 'Control Predictivo',
                'descripcion': 'Anticipación basada en curvatura futura',
                'implementacion': 'MPC con horizonte de predicción H=10',
                'efectividad': 85
            })
    
    soluciones.append({
        'titulo': 'Filtro de Trade-off Velocidad-Precisión',
        'descripcion': 'Balance dinámico entre rapidez y precisión',
        'implementacion': 'Algoritmo de optimización multiobjetivo',
        'efectividad': 80
    })
    return soluciones

def generar_datos_analisis(params):
    t_valores = np.linspace(0.1, 5.0, 101)
    datos_analisis = []
    datos_trayectoria = []
    todos_problemas = []
    
    for t in t_valores:
        x, y = folio_parametrico(t, params['a'])
        curvatura = calcular_curvatura(t, params['a'])
        velocidad_base = calcular_velocidad_adaptativa(
            curvatura, params['velocidad_maxima'], params['limite_curvatura'], params['control_adaptativo']
        )
        velocidad_suavizada = velocidad_base * (1 - params['factor_suavizado']) + \
                              params['velocidad_maxima'] * params['factor_suavizado']
        problemas = identificar_problemas(t, curvatura, velocidad_suavizada, params['a'])
        
        datos_analisis.append({
            't': t,
            'x': x,
            'y': y,
            'curvatura': curvatura,
            'velocidad': velocidad_suavizada,
            'problemas': len(problemas),
            'distancia_desde_origen': np.sqrt(x**2 + y**2)
        })
        
        datos_trayectoria.append({
            'x': x,
            'y': y,
            'problemas': len(problemas) > 0
        })
        
        todos_problemas.extend(problemas)
    
    todas_soluciones = generar_soluciones(todos_problemas)
    return datos_analisis, datos_trayectoria, todos_problemas, todas_soluciones

# Diseño de la interfaz
app.layout = html.Div(className='max-w-7xl mx-auto p-6 bg-gray-50 min-h-screen', children=[
    html.Div(className='bg-white rounded-lg shadow-lg p-6 mb-6', children=[
        html.H1('Análisis de Trayectoria Robótica - Folio de Descartes', className='text-3xl font-bold text-gray-800 mb-2'),
        html.P('Identificación automática de problemas y generación de soluciones para robots siguiendo el folio de Descartes', className='text-gray-600 mb-4'),
        
        html.Div(className='grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4 mb-6', children=[
            html.Div(className='bg-blue-50 p-4 rounded-lg', children=[
                html.Label('Parámetro a', className='block text-sm font-medium text-gray-700 mb-2'),
                dcc.Slider(id='a-slider', min=0.5, max=3.0, step=0.1, value=parametros_iniciales['a'], marks={0.5: '0.5', 3.0: '3.0'}, className='w-full'),
                html.Div(id='a-valor', className='text-sm text-gray-600')
            ]),
            html.Div(className='bg-green-50 p-4 rounded-lg', children=[
                html.Label('Velocidad Máxima', className='block text-sm font-medium text-gray-700 mb-2'),
                dcc.Slider(id='velocidad-maxima-slider', min=0.1, max=2.0, step=0.1, value=parametros_iniciales['velocidad_maxima'], marks={0.1: '0.1', 2.0: '2.0'}, className='w-full'),
                html.Div(id='velocidad-maxima-valor', className='text-sm text-gray-600')
            ]),
            html.Div(className='bg-purple-50 p-4 rounded-lg', children=[
                html.Label('Límite de Curvatura', className='block text-sm font-medium text-gray-700 mb-2'),
                dcc.Slider(id='limite-curvatura-slider', min=1.0, max=5.0, step=0.5, value=parametros_iniciales['limite_curvatura'], marks={1.0: '1.0', 5.0: '5.0'}, className='w-full'),
                html.Div(id='limite-curvatura-valor', className='text-sm text-gray-600')
            ]),
            html.Div(className='bg-orange-50 p-4 rounded-lg', children=[
                html.Label(className='flex items-center', children=[
                    dcc.Checklist(id='control-adaptativo', options=[{'label': '', 'value': 'activado'}], value=['activado'], className='mr-2'),
                    html.Span('Control Adaptativo', className='text-sm font-medium text-gray-700')
                ]),
                html.Label('Suavizado', className='block text-xs text-gray-600 mb-1 mt-2'),
                dcc.Slider(id='factor-suavizado-slider', min=0.0, max=0.5, step=0.05, value=parametros_iniciales['factor_suavizado'], marks={0.0: '0.0', 0.5: '0.5'}, className='w-full'),
                html.Div(id='factor-suavizado-valor', className='text-xs text-gray-500')
            ])
        ]),
        
        html.Div(className='flex items-center gap-4 mb-6', children=[
            html.Button(id='boton-sim', n_clicks=0, children=[
                html.Span('Iniciar Simulación', id='boton-sim-texto'),
                html.Span('▶', className='ml-2')
            ], className='flex items-center px-4 py-2 bg-green-500 hover:bg-green-600 text-white rounded-lg font-medium'),
            html.Button(id='boton-reiniciar', n_clicks=0, className='flex items-center px-4 py-2 bg-gray-500 hover:bg-gray-600 text-white rounded-lg font-medium', children=[
                html.Span('↻', className='mr-2'),
                'Reiniciar'
            ]),
            html.Div(className='flex items-center ml-auto', children=[
                html.Span('⚡', className='mr-2 text-blue-500'),
                html.Span(id='tiempo-actual', children='Tiempo: 0.00s', className='text-sm text-gray-600')
            ])
        ])
    ]),
    
    html.Div(className='grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6', children=[
        html.Div(className='bg-white rounded-lg shadow-lg p-6', children=[
            html.H3('Trayectoria del Robot', className='text-lg font-semibold mb-4'),
            dcc.Graph(id='grafico-trayectoria', style={'height': '300px'}),
            html.Div(className='mt-2 flex items-center gap-4 text-xs', children=[
                html.Div(className='flex items-center', children=[
                    html.Div(className='w-3 h-3 bg-[#3b82f6] rounded mr-1'),
                    html.Span('Trayectoria normal')
                ]),
                html.Div(className='flex items-center', children=[
                    html.Div(className='w-3 h-3 bg-[#ef4444] rounded mr-1'),
                    html.Span('Zonas problemáticas')
                ]),
                html.Div(className='flex items-center', children=[
                    html.Div(className='w-3 h-3 bg-[#10b981] rounded mr-1'),
                    html.Span('Robot actual')
                ]),
                html.Div(className='flex items-center', children=[
                    html.Div(className='w-3 h-3 bg-[#8b5cf6] rounded mr-1'),
                    html.Span('Trayectoria recorrida')
                ])
            ])
        ]),
        html.Div(className='bg-white rounded-lg shadow-lg p-6', children=[
            html.H3('Curvatura y Velocidad vs Tiempo', className='text-lg font-semibold mb-4'),
            dcc.Graph(id='grafico-curvatura-velocidad', style={'height': '300px'})
        ])
    ]),
    
    html.Div(className='grid grid-cols-1 lg:grid-cols-2 gap-6', children=[
        html.Div(className='bg-white rounded-lg shadow-lg p-6', children=[
            html.H3(id='titulo-problemas', className='text-lg font-semibold mb-4 flex items-center', children=[
                html.Span('⚠', className='mr-2 text-red-500'),
                'Problemas Identificados'
            ]),
            html.Div(id='lista-problemas', className='max-h-96 overflow-y-auto')
        ]),
        html.Div(className='bg-white rounded-lg shadow-lg p-6', children=[
            html.H3(id='titulo-soluciones', className='text-lg font-semibold mb-4 flex items-center', children=[
                html.Span('✔', className='mr-2 text-green-500'),
                'Soluciones Implementables'
            ]),
            html.Div(id='lista-soluciones', className='max-h-96 overflow-y-auto')
        ])
    ]),
    
    html.Div(className='bg-white rounded-lg shadow-lg p-6 mt-6', children=[
        html.H3('Resumen del Análisis', className='text-lg font-semibold mb-4'),
        html.Div(id='resumen-estadisticas', className='grid grid-cols-2 md:grid-cols-4 gap-4')
    ]),
    
    dcc.Interval(id='intervalo-simulacion', interval=100, disabled=True, n_intervals=0),
    dcc.Store(id='estado-simulacion', data=simulacion_inicial),
    dcc.Store(id='parametros', data=parametros_iniciales)
])

# Callbacks
@app.callback(
    [
        Output('a-valor', 'children'),
        Output('velocidad-maxima-valor', 'children'),
        Output('limite-curvatura-valor', 'children'),
        Output('factor-suavizado-valor', 'children')
    ],
    [
        Input('a-slider', 'value'),
        Input('velocidad-maxima-slider', 'value'),
        Input('limite-curvatura-slider', 'value'),
        Input('factor-suavizado-slider', 'value')
    ]
)
def actualizar_valores_sliders(a, velocidad_maxima, limite_curvatura, factor_suavizado):
    return (
        f'{a:.1f}',
        f'{velocidad_maxima:.1f} m/s',
        f'{limite_curvatura:.1f}',
        f'{factor_suavizado:.2f}'
    )

@app.callback(
    [
        Output('parametros', 'data'),
        Output('grafico-trayectoria', 'figure'),
        Output('grafico-curvatura-velocidad', 'figure'),
        Output('titulo-problemas', 'children'),
        Output('lista-problemas', 'children'),
        Output('titulo-soluciones', 'children'),
        Output('lista-soluciones', 'children'),
        Output('resumen-estadisticas', 'children')
    ],
    [
        Input('a-slider', 'value'),
        Input('velocidad-maxima-slider', 'value'),
        Input('limite-curvatura-slider', 'value'),
        Input('factor-suavizado-slider', 'value'),
        Input('control-adaptativo', 'value'),
        Input('estado-simulacion', 'data')
    ]
)
def actualizar_analisis(a, velocidad_maxima, limite_curvatura, factor_suavizado, control_adaptativo, estado_sim):
    params = {
        'a': a,
        'velocidad_maxima': velocidad_maxima,
        'limite_curvatura': limite_curvatura,
        'factor_suavizado': factor_suavizado,
        'control_adaptativo': bool(control_adaptativo)
    }
    
    datos_analisis, datos_trayectoria, problemas, soluciones = generar_datos_analisis(params)
    
    figura_trayectoria = go.Figure()
    colores = ['#ef4444' if punto['problemas'] else '#3b82f6' for punto in datos_trayectoria]
    figura_trayectoria.add_trace(go.Scatter(
        x=[punto['x'] for punto in datos_trayectoria],
        y=[punto['y'] for punto in datos_trayectoria],
        mode='markers',
        marker=dict(color=colores, size=5),
        name='Trayectoria'
    ))
    if estado_sim['posicion_robot']['x'] != 0:
        figura_trayectoria.add_trace(go.Scatter(
            x=[estado_sim['posicion_robot']['x']],
            y=[estado_sim['posicion_robot']['y']],
            mode='markers',
            marker=dict(color='#10b981', size=10),
            name='Robot actual'
        ))
    if estado_sim['trayectoria']:
        figura_trayectoria.add_trace(go.Scatter(
            x=[p['x'] for p in estado_sim['trayectoria']],
            y=[p['y'] for p in estado_sim['trayectoria']],
            mode='markers',
            marker=dict(color='#8b5cf6', size=5),
            name='Trayectoria recorrida'
        ))
    figura_trayectoria.update_layout(
        xaxis=dict(range=[-0.5, 2], title='X'),
        yaxis=dict(range=[-0.5, 2], title='Y'),
        showlegend=False,
        margin=dict(l=40, r=40, t=40, b=40)
    )
    
    figura_curvatura_velocidad = go.Figure()
    figura_curvatura_velocidad.add_trace(go.Scatter(
        x=[d['t'] for d in datos_analisis],
        y=[d['curvatura'] for d in datos_analisis],
        mode='lines',
        name='Curvatura κ',
        line=dict(color='#ef4444')
    ))
    figura_curvatura_velocidad.add_trace(go.Scatter(
        x=[d['t'] for d in datos_analisis],
        y=[d['velocidad'] for d in datos_analisis],
        mode='lines',
        name='Velocidad (m/s)',
        line=dict(color='#3b82f6')
    ))
    figura_curvatura_velocidad.update_layout(
        xaxis=dict(title='Tiempo (s)'),
        yaxis=dict(title='Valor'),
        showlegend=True,
        margin=dict(l=40, r=40, t=40, b=40)
    )
    
    titulo_problemas = [html.Span('⚠', className='mr-2 text-red-500'), f'Problemas Identificados ({len(problemas)})']
    lista_problemas = [
        html.Div(className=f"p-3 border-l-4 {'border-red-500 bg-red-50' if p['severidad'] == 'crítico' else 'border-yellow-500 bg-yellow-50'} mb-2", children=[
            html.Div(className='flex items-center mb-1', children=[
                html.Span('⚠', className=f"mr-2 {'text-red-500' if p['severidad'] == 'crítico' else 'text-yellow-500'}"),
                html.Span(p['descripcion'], className='font-semibold text-sm')
            ]),
            html.P(f"Posición: ({p['posicion']['x']:.3f}, {p['posicion']['y']:.3f})", className='text-xs text-gray-600 mb-1'),
            html.P(p['recomendacion'], className='text-xs text-blue-600')
        ]) for p in problemas[:10]
    ] if problemas else [html.P('No se detectaron problemas críticos', className='text-gray-500 text-sm')]
    
    titulo_soluciones = [html.Span('✔', className='mr-2 text-green-500'), f'Soluciones Implementables ({len(soluciones)})']
    lista_soluciones = [
        html.Div(className='p-3 border border-green-200 bg-green-50 rounded mb-2', children=[
            html.Div(className='flex items-center justify-between mb-2', children=[
                html.H4(s['titulo'], className='font-semibold text-sm text-green-800'),
                html.Div(className='flex items-center', children=[
                    html.Span('✔', className='mr-1 text-green-600'),
                    html.Span(f"{s['efectividad']}%", className='text-xs text-green-600')
                ])
            ]),
            html.P(s['descripcion'], className='text-xs text-gray-700 mb-1'),
            html.P(s['implementacion'], className='text-xs text-blue-600 font-mono')
        ]) for s in soluciones
    ]
    
    conteo_criticos = sum(1 for p in problemas if p['severidad'] == 'crítico')
    conteo_advertencias = sum(1 for p in problemas if p['severidad'] == 'advertencia')
    efectividad_promedio = round(sum(s['efectividad'] for s in soluciones) / len(soluciones)) if soluciones else 0
    resumen_estadisticas = [
        html.Div(className='text-center', children=[
            html.Div(f'{conteo_criticos}', className='text-2xl font-bold text-red-500'),
            html.Div('Problemas Críticos', className='text-sm text-gray-600')
        ]),
        html.Div(className='text-center', children=[
            html.Div(f'{conteo_advertencias}', className='text-2xl font-bold text-yellow-500'),
            html.Div('Advertencias', className='text-sm text-gray-600')
        ]),
        html.Div(className='text-center', children=[
            html.Div(f'{len(soluciones)}', className='text-2xl font-bold text-green-500'),
            html.Div('Soluciones Disponibles', className='text-sm text-gray-600')
        ]),
        html.Div(className='text-center', children=[
            html.Div(f'{efectividad_promedio}%', className='text-2xl font-bold text-blue-500'),
            html.Div('Efectividad Promedio', className='text-sm text-gray-600')
        ])
    ]
    
    return (
        params,
        figura_trayectoria,
        figura_curvatura_velocidad,
        titulo_problemas,
        lista_problemas,
        titulo_soluciones,
        lista_soluciones,
        resumen_estadisticas
    )

@app.callback(
    [
        Output('boton-sim', 'className'),
        Output('boton-sim-texto', 'children'),
        Output('intervalo-simulacion', 'disabled'),
        Output('estado-simulacion', 'data'),
        Output('tiempo-actual', 'children')
    ],
    [
        Input('boton-sim', 'n_clicks'),
        Input('boton-reiniciar', 'n_clicks'),
        Input('intervalo-simulacion', 'n_intervals'),
        Input('parametros', 'data')
    ],
    [
        State('estado-simulacion', 'data')
    ]
)
def controlar_simulacion(sim_clicks, reiniciar_clicks, n_intervalos, params, estado_sim):
    ctx = callback_context
    id_activado = ctx.triggered[0]['prop_id'].split('.')[0]
    
    if id_activado == 'boton-reiniciar':
        return (
            'flex items-center px-4 py-2 bg-green-500 hover:bg-green-600 text-white rounded-lg font-medium',
            'Iniciar Simulación',
            True,
            simulacion_inicial,
            'Tiempo: 0.00s'
        )
    
    if id_activado == 'boton-sim':
        esta_corriendo = not estado_sim['esta_corriendo']
        return (
            f"flex items-center px-4 py-2 {'bg-red-500 hover:bg-red-600' if esta_corriendo else 'bg-green-500 hover:bg-green-600'} text-white rounded-lg font-medium",
            'Pausar Simulación' if esta_corriendo else 'Iniciar Simulación',
            not esta_corriendo,
            {**estado_sim, 'esta_corriendo': esta_corriendo},
            f"Tiempo: {estado_sim['tiempo_actual']:.2f}s"
        )
    
    if id_activado == 'intervalo-simulacion' and estado_sim['esta_corriendo']:
        nuevo_tiempo = estado_sim['tiempo_actual'] + 0.05
        if nuevo_tiempo > 5.0:
            return (
                'flex items-center px-4 py-2 bg-green-500 hover:bg-green-600 text-white rounded-lg font-medium',
                'Iniciar Simulación',
                True,
                {**estado_sim, 'esta_corriendo': False},
                f"Tiempo: {nuevo_tiempo:.2f}s"
            )
        
        t = 0.1 + nuevo_tiempo
        x, y = folio_parametrico(t, params['a'])
        nueva_trayectoria = estado_sim['trayectoria'] + [{'x': x, 'y': y}]
        nueva_trayectoria = nueva_trayectoria[-50:]
        
        return (
            'flex items-center px-4 py-2 bg-red-500 hover:bg-red-600 text-white rounded-lg font-medium',
            'Pausar Simulación',
            False,
            {
                **estado_sim,
                'tiempo_actual': nuevo_tiempo,
                'posicion_robot': {'x': x, 'y': y},
                'trayectoria': nueva_trayectoria
            },
            f"Tiempo: {nuevo_tiempo:.2f}s"
        )
    
    return (
        f"flex items-center px-4 py-2 {'bg-red-500 hover:bg-red-600' if estado_sim['esta_corriendo'] else 'bg-green-500 hover:bg-green-600'} text-white rounded-lg font-medium",
        'Pausar Simulación' if estado_sim['esta_corriendo'] else 'Iniciar Simulación',
        not estado_sim['esta_corriendo'],
        estado_sim,
        f"Tiempo: {estado_sim['tiempo_actual']:.2f}s"
    )

if __name__ == '__main__':
    app.run(debug=True)
