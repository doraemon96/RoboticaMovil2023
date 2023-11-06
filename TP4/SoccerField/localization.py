""" Written by Brian Hou for CSE571: Probabilistic Robotics (Winter 2019)
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt

from utils import minimized_angle, plot_field, plot_robot, plot_path
from soccer_field import Field
import policies
from ekf import ExtendedKalmanFilter
from pf import ParticleFilter

import json


def localize(env, policy, filt, x0, num_steps, plot=False):
    # Collect data from an entire rollout
    states_noisefree, states_real, action_noisefree, obs_noisefree, obs_real = \
        env.rollout(x0, policy, num_steps)
    states_filter = np.zeros(states_real.shape)
    states_filter[0, :] = x0.ravel()

    errors = np.zeros((num_steps, 3))
    position_errors = np.zeros(num_steps)
    mahalanobis_errors = np.zeros(num_steps)

    if plot:
        fig = env.get_figure()

    for i in range(num_steps):
        x_real = states_real[i + 1, :].reshape((-1, 1))
        u_noisefree = action_noisefree[i, :].reshape((-1, 1))
        z_real = obs_real[i, :].reshape((-1, 1))
        marker_id = env.get_marker_id(i)

        if filt is None:
            mean, cov = x_real, np.eye(3)
        else:
            # filters only know the action and observation
            mean, cov = filt.update(env, u_noisefree, z_real, marker_id)
        states_filter[i + 1, :] = mean.ravel()

        if plot:
            fig.clear()
            plot_field(env, marker_id)
            plot_robot(env, x_real, z_real)
            plot_path(env, states_noisefree[:i + 1, :], 'g', 0.5)
            plot_path(env, states_real[:i + 1, :], 'b')
            if filt is not None:
                plot_path(env, states_filter[:i + 1, :2], 'r')
            fig.canvas.flush_events()

        errors[i, :] = (mean - x_real).ravel()
        errors[i, 2] = minimized_angle(errors[i, 2])
        position_errors[i] = np.linalg.norm(errors[i, :2])

        cond_number = np.linalg.cond(cov)
        if cond_number > 1e12:
            # print('Badly conditioned cov (setting to identity):', cond_number)
            # print(cov)
            cov = np.eye(3)
        mahalanobis_errors[i] = \
            errors[i:i + 1, :].dot(np.linalg.inv(cov)).dot(errors[i:i + 1, :].T)[0][0]

    mean_position_error = position_errors.mean()
    mean_mahalanobis_error = mahalanobis_errors.mean()
    anees = mean_mahalanobis_error / 3

    if filt is not None:
        print('Mean position error:', mean_position_error)
        print('Mean Mahalanobis error:', mean_mahalanobis_error)
        print('ANEES:', anees)

    if plot:
        plt.show(block=True)

    return mean_position_error, mean_mahalanobis_error, anees


def setup_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'filter_type', choices=('none', 'ekf', 'pf'),
        help='filter to use for localization')
    parser.add_argument(
        '--plot', action='store_true',
        help='turn on plotting')
    parser.add_argument(
        '--seed', type=int,
        help='random seed')
    parser.add_argument(
        '--num-steps', type=int, default=200,
        help='timesteps to simulate')

    # Noise scaling factors
    parser.add_argument(
        '--data-factor', type=float, default=1,
        help='scaling factor for motion and observation noise (data)')
    parser.add_argument(
        '--filter-factor', type=float, default=1,
        help='scaling factor for motion and observation noise (filter)')
    parser.add_argument(
        '--num-particles', type=int, default=100,
        help='number of particles (particle filter only)')

    parser.add_argument(
        '--num-runs', type=int, default=1,
        help='number of runs')
    parser.add_argument(
        '--loop-factors', action='store_true',
        help='habilita variaciones de alpha y beta sobre el vector r (hardcoded)'
    )
    parser.add_argument(
        '--no-dataloop', action='store_true',
        help='deshabilita variaciones en los data factors'
    )
    parser.add_argument(
        '--loop-particles', action='store_true',
        help='habilita variaciones en el numero de partículas'
    )

    return parser


def run_simulation(args):
    if args.seed is not None:
        np.random.seed(args.seed)

    alphas = np.array([0.05 ** 2, 0.005 ** 2, 0.1 ** 2, 0.01 ** 2])
    beta = np.diag([np.deg2rad(5) ** 2])

    env = Field(args.data_factor * alphas, args.data_factor * beta)
    policy = policies.OpenLoopRectanglePolicy()

    initial_mean = np.array([180, 50, 0]).reshape((-1, 1))
    initial_cov = np.diag([10, 10, 1])

    if args.filter_type == 'none':
        filt = None
    elif args.filter_type == 'ekf':
        filt = ExtendedKalmanFilter(
            initial_mean,
            initial_cov,
            args.filter_factor * alphas,
            args.filter_factor * beta
        )
    elif args.filter_type == 'pf':
        filt = ParticleFilter(
            initial_mean,
            initial_cov,
            args.num_particles,
            args.filter_factor * alphas,
            args.filter_factor * beta
        )

    # You may want to edit this line to run multiple localization experiments.
    return localize(env, policy, filt, initial_mean, args.num_steps, args.plot)


def default(obj):  ## ESTA FUNNCION SIRVE PARA GUARDAR NUMPY ARRAYS EN EL JSON
    if type(obj).__module__ == np.__name__:
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        else:
            return obj.item()
    raise TypeError('Unknown type:', type(obj))


if __name__ == '__main__':
    args = setup_parser().parse_args()
    if args.loop_factors:
        r = np.array(['1/64', '1/16', '1/4', '1', '4', '16', '64'])
    else:
        r = np.array(['1'])

    if args.loop_particles and args.filter_type == 'pf':
        particles = np.array(['20', '50', '500'])
    else:
        particles = np.array(['100'])

    # Este diccionario se guarda en un archivo al finalizar las corridas
    
    savedict = {'filter_type': args.filter_type,
                'runs': args.num_runs,
                'r': r,
                'no-dataloop': args.no_dataloop,
                'results': {}
                }
    if args.loop_particles and args.filter_type == 'pf':
        savedict['particles'] = particles

    data_factor_original = args.data_factor
    filter_factor_original = args.filter_factor

    count = 0

    for f in r:
        print('-' * 80)
        print('-' * 80)
        print('Setting up simulation parameters...')
        savedict['results'][f] = {}

        if not args.no_dataloop:
            args.data_factor = data_factor_original * eval(f)
        args.filter_factor = filter_factor_original * eval(f)
        print('Factor multiplier: ', f)
        print('Data factor:', args.data_factor)
        print('Filter factor:', args.filter_factor)
        savedict['results'][f]['data_factor'] = args.data_factor
        savedict['results'][f]['filter_factor'] = args.filter_factor

        savedict['results'][f]['runs'] = {}
        for p in particles:
            args.num_particles = eval(p) 
            E = np.zeros((args.num_runs, 3))
            for i in range(args.num_runs):
                count = count + 1
                print('-' * 80)
                print('Running simulation {} of {}'.format(count, 
                                                            args.num_runs*len(r)*len(particles)))
                E[i, :] = run_simulation(args)
            if args.loop_particles and args.filter_type == 'pf':
                savedict['results'][f]['runs'][p] = E
            else:
                savedict['results'][f]['runs'] = E

    if not args.filter_type == 'none':
        with open('results-{filter}{factorloop}{dataloop}{partloop}.json'.format(
                    filter=args.filter_type,
                    factorloop=('-fl' if args.loop_factors else ''),
                    dataloop=('-ndl' if args.no_dataloop else ''),
                    partloop=('-pl' if args.loop_particles and args.filter_type == 'pf' else '')),
                    "w") as f:
            # f.write(str(savedict))
            json.dump(savedict, f, default=default)

    print("Final Results: {!s}".format(savedict['results']))
