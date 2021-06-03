import random
import json
import os
import copy
import numpy as np

file_path = os.path.dirname(os.path.abspath(__file__))

class GeneticAlgorithm(object):

    def __init__(self, max_time, load=True):
        self.max_time = max_time
        self.generation_size = 100
        self.generation_num = 0
        self.generation = []

        self.n_crossover_mean = self.generation_size * 2
        self.n_crossover_swap = self.generation_size * 2
        self.n_mutation = self.generation_size * 2

        self.param_keys = [
            "prey_weight",
            "parallel_weight",
            "away_weight",
            "min_turn_only_angle",
            "base_speed",
            "scaled_speed",
            "angle_adjust_rate"
        ]

        if load:
            self.load(0)
            self.set_subject()


    def get_filename(self, generation_num):
        return f'generation_{generation_num}.json'


    def random_params(self):
        params = {}
        for key in self.param_keys:
            params[key] = np.random.uniform()
        return params


    def init_and_save_generation(self):

        for i in range(self.generation_size):
            v = {
                "params": self.random_params(),
                "score": 1 / self.generation_size,
                "tested": False
            }
            self.generation.append(v)

        self.save()


    def count_tested(self):
        c = 0
        for v in self.generation:
            if v["tested"]:
                c += 1
        return c


    def print_progress(self):
        tested = self.count_tested()
        size = self.generation_size
        print(f'Generation {self.generation_num} Progress: {tested}/{size}')


    def set_subject(self):
        for v in self.generation:
            if not v["tested"]:
                self.subject = v
                return True
        
        return False


    def get_params(self):
        return self.subject["params"]


    def set_score_by_time(self, time):
        time_param = self.max_time * 1.03
        score = min(1, max(0, (time_param - time) / time_param))
        self.set_score(score)


    def set_score(self, score):
        self.subject["score"] = score
        self.subject["tested"] = True

        if not self.set_subject():
            self.generate_next_generation()
            self.set_subject()


    def regularize_scores(self):
        # scores should total 1 like in particle filter
        scores_tot = 0

        for v in self.generation:
            scores_tot += v["score"]
        
        for v in self.generation:
            v["score"] = v["score"] / scores_tot


    def crossover_mean(self, v1, v2):
        key = np.random.choice(self.param_keys)
        p1 = v1["params"][key]
        p2 = v2["params"][key]
        avg = (p1 + p2) / 2
        v1["params"][key] = avg
        v2["params"][key] = avg


    def crossover_swap(self, v1, v2):
        key = np.random.choice(self.param_keys)
        temp = v1["params"][key]
        v1["params"][key] = v2["params"][key]
        v2["params"][key] = temp


    def mutation(self, v):
        key = np.random.choice(self.param_keys)
        v["params"][key] += np.random.uniform(-.1, .1)


    def generate_next_generation(self):
        self.regularize_scores()
        self.save()

        self.generation_num += 1
        # randomly select based on scores, then do mutation and crossover
        probs = list(map(lambda x: x["score"], self.generation))

        next_generation = np.random.choice(self.generation, size=self.generation_size, p=probs).tolist()

        for i in range(len(next_generation)):
            next_generation[i] = copy.deepcopy(next_generation[i])
            next_generation[i]["tested"] = False
            next_generation[i]["score"] = 1 / self.generation_size

        for i in range(self.n_crossover_mean):
            v1 = np.random.choice(next_generation)
            v2 = np.random.choice(next_generation)
            self.crossover_mean(v1, v2)
        
        for i in range(self.n_crossover_swap):
            v1 = np.random.choice(next_generation)
            v2 = np.random.choice(next_generation)
            self.crossover_swap(v1, v2)
        
        for i in range(self.n_mutation):
            v = np.random.choice(next_generation)
            self.mutation(v)

        self.generation = next_generation
        return next_generation


    def save(self):
        filename = self.get_filename(self.generation_num)
        full_data = {
            "generation": self.generation,
            "generation_num": self.generation_num
        }
        with open(file_path + '/data/' + filename, 'w') as f:
            json.dump(full_data, f)


    def load(self, generation_num=0):
        filename = self.get_filename(generation_num)
        with open(file_path + '/data/' + filename, 'r') as f:
            data = json.load(f)
            self.generation = data["generation"]
            self.generation_num = data["generation_num"]

    def modify_gen(self):
        for v in self.generation:
            v["tested"] = False
        self.generation = self.generation[:100]

if __name__ == '__main__':
    ga = GeneticAlgorithm(0, False)
    ga.load(0)
    # ga.modify_gen()
    # ga.save()

    # ga.init_and_save_generation()
    # ga.generate_next_generation()
    # ga.generate_next_generation()
