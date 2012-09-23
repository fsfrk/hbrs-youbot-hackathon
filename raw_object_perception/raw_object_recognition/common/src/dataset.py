import os
import yaml
import numpy as np


class Dataset:
    def __init__(self, base_folder, dataset_name):
        self.base_folder = base_folder
        self.dataset_folder = os.path.join(base_folder, dataset_name)
        if not os.path.isdir(self.dataset_folder):
            os.mkdir(self.dataset_folder)
        self.objects_yaml = os.path.join(base_folder, 'objects.yaml')
        self.objects = list(yaml.load_all(open(self.objects_yaml)))

    def store(self, object_id, dim, pts, color_mean, color_median):
        f = open(self.object_data_filename(object_id), 'a')
        d = sorted([dim.x, dim.y, dim.z], reverse=True)
        f.write('%.3f %.3f %.3f %i %.2f %.2f\n' % (d[0], d[1], d[2], pts,
                                                   color_mean, color_median))
        f.close()

    def object_data_filename(self, object_id):
        return os.path.join(self.dataset_folder, object_id + '.dat')

    def load(self, objects):
        data = []
        for i, obj in enumerate(objects):
            d = np.loadtxt(self.object_data_filename(obj))
            d = np.hstack((d, np.ones((len(d), 1)) * i))
            data.append(d)
        return np.vstack(data)
