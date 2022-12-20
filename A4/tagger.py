import os
import sys
import time
import numpy as np
from collections import Counter, defaultdict

UNIVERSAL_TAGS = [
    "VERB",
    "NOUN",
    "PRON",
    "ADJ",
    "ADV",
    "ADP",
    "CONJ",
    "DET",
    "NUM",
    "PRT",
    "X",
    ".",
]

N_tags = len(UNIVERSAL_TAGS)

def read_data_train(path):
    return [tuple(line.split(' : ')) for line in open(path, 'r').read().split('\n')[:-1]]

def read_data_test(path):
    return open(path, 'r').read().split('\n')[:-1]

def read_data_ind(path):
    return [int(line) for line in open(path, 'r').read().split('\n')[:-1]]

def write_results(path, results):
    with open(path, 'w') as f:
        f.write('\n'.join(results))

def train_HMM(train_file_name):
    """
    Estimate HMM parameters from the provided training data.

    Input: Name of the training files. Two files are provided to you:
            - file_name.txt: Each line contains a pair of word and its Part-of-Speech (POS) tag
            - fila_name.ind: The i'th line contains an integer denoting the starting index of the i'th sentence in the text-POS data above

    Output: Three pieces of HMM parameters stored in LOG PROBABILITIES :

            - prior:        - An array of size N_tags
                            - Each entry corresponds to the prior log probability of seeing the i'th tag in UNIVERSAL_TAGS at the beginning of a sequence
                            - i.e. prior[i] = log P(tag_i)

            - transition:   - A 2D-array of size (N_tags, N_tags)
                            - The (i,j)'th entry stores the log probablity of seeing the j'th tag given it is a transition coming from the i'th tag in UNIVERSAL_TAGS
                            - i.e. transition[i, j] = log P(tag_j|tag_i)

            - emission:     - A dictionary type containing tuples of (str, str) as keys
                            - Each key in the dictionary refers to a (TAG, WORD) pair
                            - The TAG must be an element of UNIVERSAL_TAGS, however the WORD can be anything that appears in the training data
                            - The value corresponding to the (TAG, WORD) key pair is the log probability of observing WORD given a TAG
                            - i.e. emission[(tag, word)] = log P(word|tag)
                            - If a particular (TAG, WORD) pair has never appeared in the training data, then the key (TAG, WORD) should not exist.

    Hints: 1. Think about what should be done when you encounter those unseen emission entries during deccoding.
           2. You may find Python's builtin Counter object to be particularly useful
    """

    """
    1. prior_dict -> Record the tags appear in the first word of a sentence.
    2. trans_dict -> Record the numbers of transition between (i,j)
    3. emiss_dict -> Record the numbers of all (TAG , WORD)
    4. tag_dict -> Record the numbers of each TAG.
    """
    prior_dict = defaultdict(int)
    trans_dict = defaultdict(int)
    emiss_dict = defaultdict(int)
    tag_dict = defaultdict(int)

    # Get the train data into a list
    pos_data = read_data_train(train_file_name+'.txt')
    sent_inds = read_data_ind(train_file_name+'.ind')

    # Results
    prior = np.empty((N_tags) , dtype = float)
    transition = np.empty((N_tags , N_tags) , dtype = float)
    emission = defaultdict(float)

    #Prior
    count = 0
    for start in sent_inds:
        word = pos_data[int(start)]
        prior_dict[word[1]] += 1
        count += 1
    for i in range(N_tags):
        if prior_dict[UNIVERSAL_TAGS[i]] / count == 0:
            # Avoid log(0)
            prior[i] = float("-inf")
        else:
            prior[i] = np.log(prior_dict[UNIVERSAL_TAGS[i]] / count)

    #Transition
    """
    Step 1. : Use trans_dict to record the numbers of all transitions.
    Step 2. : Calculate the probability of the transitions.
    """
    #Step 1.
    for i in range(1 , len(sent_inds)):
        #print('({},{})'.format(sent_inds[i-1] , sent_inds[i]))
        for j in range(sent_inds[i-1] , sent_inds[i]-1):
            FROM = pos_data[j]
            TO = pos_data[j+1]
            trans_dict['({},{})'.format(FROM[1] , TO[1])] += 1

    for k in range(sent_inds[-1] , len(pos_data) - 1 ):
        FROM = pos_data[k]
        TO = pos_data[k+1]
        trans_dict['({},{})'.format(FROM[1] , TO[1])] += 1

    #print(sorted(trans_dict.items() , key= lambda d:d[1] ))

    #Step 2.
    for i in range(N_tags):
        sum_up = 0
        for j in range(N_tags):
            key = '('+UNIVERSAL_TAGS[i]+','+ UNIVERSAL_TAGS[j]+')'
            sum_up += trans_dict[key]
        for k in range(N_tags):
            key = '('+UNIVERSAL_TAGS[i]+','+ UNIVERSAL_TAGS[k]+')'
            # Avoid log(0)
            if trans_dict[key] / sum_up == 0:
                transition[i][k] = float("-inf")
            else:
                transition[i][k] = np.log(trans_dict[key] / sum_up)


    #Emission
    """
    Step 1. : Use emiss_dict to record the numbers of (TAG ,WORD) and use tag_dict to record the numbers of each TAG.
    Step 2. : Calculate the probability of observing WORD given a TAG.
    """
    for line in pos_data:
        emiss_dict[(line[0] , line[1])] += 1
        tag_dict[line[1]] += 1
    for key in emiss_dict:
        emission[(key[1] , key[0])] = np.log(emiss_dict[key]/tag_dict[key[1]])

    return prior, transition, emission

"""
I would use Viterbi algorithm to predict the TAGs, please refer to the pseudo code.
"""
def test(pos_data , sent_inds , prior , transition , emission):
    results = []
    # min_ = float("inf")
    # for key in emission:
    #     if emission[key] < min_:
    #         min_ = emission[key]
    # print(min_)

    """
    Since the index data has not recorded the end position of a sentence, I would append the last index to the sent_inds.
    Basically, I would use the Viterbi algorithm to predict all the sentences.
    """
    # Make index data easier to use
    sent_inds.append(len(pos_data))

    #Viterbi Algorithm
    for i in range(1 , len(sent_inds)):
        pos = 0
        prob_trellis = np.empty((N_tags , sent_inds[i] - sent_inds[i-1]) , dtype = float)
        path_trellis = np.empty((N_tags , sent_inds[i] - sent_inds[i-1]) , dtype = list)
        #Determine trellis values for X1
        for j in range(sent_inds[i-1] , sent_inds[i]):
            word = pos_data[j]
            if pos == 0:
                for s in range(N_tags):
                    # Hint 1
                    if (UNIVERSAL_TAGS[s],word) not in emission:
                        emission[(UNIVERSAL_TAGS[s],word)] = -9.68 # min_
                    prob_trellis[s][0] = prior[s] + emission[(UNIVERSAL_TAGS[s] , word)]
                    path_trellis[s][0] = [UNIVERSAL_TAGS[s]]
        #For X2 - Xt find each current state's most likely prior state x.
            else:
                for s in range(N_tags):
                    # Hint 1
                    if (UNIVERSAL_TAGS[s],word) not in emission:
                        emission[(UNIVERSAL_TAGS[s],word)] = -9.68 # min_
                    MAX = float("-inf")
                    max_x = 0
                    #Arg max
                    for x in range(N_tags):
                        #print(prob_trellis[x][j-1])
                        if float(prob_trellis[x][pos-1]) + float(transition[x][s]) + float(emission[(UNIVERSAL_TAGS[s],word)]) > MAX :
                            MAX = float(prob_trellis[x][pos-1]) + float(transition[x][s]) + float(emission[(UNIVERSAL_TAGS[s],word)])
                            max_x = x
                    prob_trellis[s][pos] = float(prob_trellis[max_x][pos-1]) + float(transition[max_x][s]) + float(emission[(UNIVERSAL_TAGS[s] , word)])
                    path_trellis[s][pos] = path_trellis[max_x][pos-1] + [UNIVERSAL_TAGS[s]]
            pos += 1
        # Choose the highest probability of state in the last word to be the best prediction.
        max_val = float("-inf")
        best_pred = []
        for i in range(N_tags):
            if prob_trellis[i][-1] > max_val:
                max_val = prob_trellis[i][-1]
                best_pred = path_trellis[i][-1]
        for pred in best_pred:
            results.append(pred)

    print("Finish Prediction!")
    return results

def tag(train_file_name, test_file_name):
    """
    Train your HMM model, run it on the test data, and finally output the tags.

    Your code should write the output tags to (test_file_name).pred, where each line contains a POS tag as in UNIVERSAL_TAGS
    """
    start_time = time.time()
    prior, transition, emission = train_HMM(train_file_name)

    pos_data = read_data_test(test_file_name+'.txt')
    sent_inds = read_data_ind(test_file_name+'.ind')

    results = test(pos_data , sent_inds , prior , transition , emission)

    write_results(test_file_name+'.pred', results)
    end_time = time.time()

    print("It takes {} seconds.".format(round(end_time - start_time)))

if __name__ == '__main__':
    # Run the tagger function.
    print("Starting the tagging process.")

    # Tagger expects the input call: "python3 tagger.py -d <training file> -t <test file>"
    # E.g. python3 tagger.py -d data/train-public -t data/test-public-small
    parameters = sys.argv
    train_file_name = parameters[parameters.index("-d")+1]
    test_file_name = parameters[parameters.index("-t")+1]

    # Start the training and tagging operation.
    tag (train_file_name, test_file_name)
