import argparse
import json
import numpy as np
import matplotlib.pyplot as plt

def show_results(data):
    figdict = {}
    number_runs = data['runs']
    xticks = np.arange(1, number_runs + 1)

    meanMPE = {}  # VECTORES DE ERRORES MEDIO EN FUNCION DE R
    meanMME = {}  # VECTORES DE ERRORES MEDIO EN FUNCION DE R
    meanANEES = {}  # VECTORES DE ERRORES MEDIO EN FUNCION DE R

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
            plt.xlabel('r')
            plt.ylabel("MPE")
            plt.grid()

            figdict['r_MME'] = plt.figure()
            plt.title("Filter: {filter} - factor vs Mean Mahalanobis Error{dl}".
                      format(filter=data['filter_type'],
                             dl='\n Fixed Data Factor' if data['no-dataloop'] else ''))
            plt.xlabel('r')
            plt.ylabel("MME")
            plt.grid()

            figdict['r_ANEES'] = plt.figure()
            plt.title("Filter: {filter} - factor vs ANEES{dl}".
                      format(filter=data['filter_type'],
                             dl='\n Fixed Data Factor' if data['no-dataloop'] else ''))
            plt.xlabel('r')
            plt.ylabel("ANEES")
            plt.grid()

    for r in data['r']:
        runDataFull = data['results'][r]['runs']
        for p in data.get('particles', '0'):
            if "particles" in data:
                runData = np.array(runDataFull[p])
            else:
                runData = np.array(runDataFull)
            if not p in meanMPE:
                meanMPE[p]=[]
                meanMME[p]=[]
                meanANEES[p]=[]
            meanMPE[p].append(np.mean(runData[:, 0]))
            meanMME[p].append(np.mean(runData[:, 1]))
            meanANEES[p].append(np.mean(runData[:, 2]))
            if number_runs > 1:
                plt.figure(figdict['run_MPE'])
                plt.plot(xticks, runData[:, 0], label='r:{r}{p}'.format(r=r, p = ', ' + p if "particles" in data else ''))
                plt.legend()

                plt.figure(figdict['run_MME'])
                plt.plot(xticks, runData[:, 1], label='r:{r}{p}'.format(r=r, p = ', ' + p if "particles" in data else ''))
                plt.legend()

                plt.figure(figdict['run_ANEES'])
                plt.plot(xticks, runData[:, 2], label='r:{r}{p}'.format(r=r, p = ', p: ' + p if "particles" in data else ''))
                plt.legend()

    if len(data['r']) > 1:
        for p in data.get('particles', '0'):
            plt.figure(figdict['r_MPE'])
            plt.plot(data['r'], meanMPE[p], label='p: {p}'.format(p = p if "particles" in data else ''))
            if "particles" in data:
                plt.legend()
                
            plt.figure(figdict['r_MME'])
            plt.plot(data['r'], meanMME[p], label='p: {p}'.format(p = p if "particles" in data else ''))
            if "particles" in data:
                plt.legend()

            plt.figure(figdict['r_ANEES'])
            plt.plot(data['r'], meanANEES[p], label='p: {p}'.format(p = p if "particles" in data else ''))
            if "particles" in data:
                plt.legend()

    plt.show(block=True)


def setup_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--data', type=str, default="", help='results file')
    return parser


if __name__ == '__main__':
    args = setup_parser().parse_args()

    if not args.data == "":
        file = open(args.data, "r")
        data = json.load(file)
        file.close()
        show_results(data)
