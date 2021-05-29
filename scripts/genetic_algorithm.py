import random
import json
import os

file_path = os.path.dirname(os.path.abspath(__file__))

class GeneticAlgorithm(object):

    def __init__(self, load=True):
        self.generation_size = 100
        self.generation_num = 0
        self.generation = []

        if load:
            self.load(0)
            self.set_subject()


    def get_filename(self, generation_num):
        return f'generation_{generation_num}.json'


    def random_params(self):
        params = {
            "prey_weight": 1,
            "parallel_weight": 0,
            "away_weight": 0,
            "min_turn_only_angle": 1.2,
            "base_speed": 0.3,
            "scaled_speed": 0,
            "angle_adjust_rate": 0.5
        }
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


    def set_subject(self):
        for v in self.generation:
            if not v["tested"]:
                self.subject = v
                return True
        
        return False


    def get_params(self):
        return self.subject["params"]


    def set_score_by_time(self, time):
        return


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


    def generate_next_generation(self):
        self.regularize_scores()
        self.save()

        self.generation_num += 1
        # randomly select based on scores, then do mutation and crossover
        return


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

if __name__ == '__main__':
    ga = GeneticAlgorithm(False)
    # ga.load(0)
    ga.init_and_save_generation()
