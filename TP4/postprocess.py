import argparse
import json
import numpy as np
import matplotlib.pyplot as plt

pf_data = None
ekf_data = None


def show_results(data):
    figdict = {}
    number_runs = data['runs']
    xticks = np.arange(1, number_runs + 1)

    meanMPE = []  # VECTORES DE ERRORES MEDIO EN FUNCION DE R
    meanMME = []  # VECTORES DE ERRORES MEDIO EN FUNCION DE R
    meanANEES = []  # VECTORES DE ERRORES MEDIO EN FUNCION DE R

    if number_runs > 1:
        figdict['run_MPE'] = plt.figure()
        plt.title("Filter: {filter} - Mean Position Error".
                  format(filter=data['filter_type']))
        plt.xticks(xticks)
        plt.xlabel('Run number')
        plt.ylabel("MPE")
        plt.grid()

        figdict['run_MME'] = plt.figure()
        plt.title("Filter: {filter} - Mean Mahalanobis Error".
                  format(filter=data['filter_type']))
        plt.xticks(xticks)
        plt.xlabel('Run number')
        plt.ylabel("MME")
        plt.grid()

        figdict['run_ANEES'] = plt.figure()
        plt.title("Filter: {filter} - ANEES".
                  format(filter=data['filter_type']))
        plt.xticks(xticks)
        plt.xlabel('Run number')
        plt.ylabel("ANEES")
        plt.grid()

        if len(data['r']) > 1:
            figdict['r_MPE'] = plt.figure()
            plt.title("Filter: {filter} - factor vs Mean Position Error{dl}".
                      format(filter=data['filter_type'],
                             dl='\n Fixed Data Factor' if data['no-dataloop'] else ''))
            # plt.xticks(xticks)
            plt.xlabel('r')
            plt.ylabel("MPE")
            plt.grid()

            figdict['r_MME'] = plt.figure()
            plt.title("Filter: {filter} - factor vs Mean Mahalanobis Error{dl}".
                      format(filter=data['filter_type'],
                             dl='\n Fixed Data Factor' if data['no-dataloop'] else ''))
            # plt.xticks(xticks)
            plt.xlabel('r')
            plt.ylabel("MME")
            plt.grid()

            figdict['r_ANEES'] = plt.figure()
            plt.title("Filter: {filter} - factor vs ANEES{dl}".
                      format(filter=data['filter_type'],
                             dl='\n Fixed Data Factor' if data['no-dataloop'] else ''))
            # plt.xticks(xticks)
            plt.xlabel('r')
            plt.ylabel("ANEES")
            plt.grid()

    for r in data['r']:
        # r = data['r'][0]
        runData = np.array(data['results'][r]['runs'])
        meanMPE.append(np.mean(runData[:, 0]))
        meanMME.append(np.mean(runData[:, 1]))
        meanANEES.append(np.mean(runData[:, 2]))
        # number_runs = runData.shape[0]
        if number_runs > 1:
            plt.figure(figdict['run_MPE'])
            plt.plot(xticks, runData[:, 0], label='r:{r}'.format(r=r))
            plt.legend()

            plt.figure(figdict['run_MME'])
            plt.plot(xticks, runData[:, 1], label='r:{r}'.format(r=r))
            plt.legend()

            plt.figure(figdict['run_ANEES'])
            plt.plot(xticks, runData[:, 2], label='r:{r}'.format(r=r))
            plt.legend()
            # plt.plot(xticks, runData[:, 1], label='MME')
            # plt.plot(xticks, runData[:, 2], label='ANEES')

        # #plt.title("Filter: {filter} - DF: {datafactor} - FF: {filterfactor}".
        #           format(filter=data['filter_type'],
        #                  datafactor=data['results'][r]['data_factor'],
        #                  filterfactor=data['results'][r]['filter_factor']))

    if len(data['r']) > 1:
        plt.figure(figdict['r_MPE'])
        plt.plot(data['r'], meanMPE)

        plt.figure(figdict['r_MME'])
        plt.plot(data['r'], meanMME)

        plt.figure(figdict['r_ANEES'])
        plt.plot(data['r'], meanANEES)
        # plt.legend()

    plt.show(block=True)


def setup_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--data', type=str, default="", help='results file')
    # parser.add_argument(
    #     '--ekf-data', type=str, default="", help='results file for extended kalman filter')

    return parser


if __name__ == '__main__':
    args = setup_parser().parse_args()

    if not args.data == "":
        file = open(args.data, "r")
        data = json.load(file)
        file.close()
        show_results(data)
