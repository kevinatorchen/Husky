import numpy as np

MAX_MATCH_DIST = 0.5

MAX_RANGE = 10


class DataAssociator:

    def __init__(self, huskySlam):
        self.slamInstance = huskySlam

    def joint_compat_score(self, hypothesis, Z):
        """
        compute the joint compat of the pairing
        this will be done by :
            joint_compat_score = D
            D**2 = h^t C^-1 h
            C = HPH^t + GSG^t
            where h is the innovation of the pairings
        :param hypothesis: id_x and id_z pairs
        :return: D
        """
        # recompose the X vector and the Z vector
        # compute C matrix
        # compute D from C^-1
        res = 0
        for row in hypothesis:
            if row[0, 0] != -1:
                print("row:"+str(row))
                id_x = row[0, 0]
                id_z = row[0, 1]
                res += np.sum(
                    np.absolute(Z[id_z: id_z + 2, 0] - self.slamInstance.X[id_x: id_x + 2, 0]))
            else:
                res += 2 * ((2 * MAX_MATCH_DIST) ** 2)
        return res  # sum([(lx_x - lz_x) ** 2 + (lx_y - lz_y) ** 2 for ((lx_x, lx_y), (lz_x, lz_y)) in pairings])

    def individual_compatibility_score(self, id_x, id_z, Z):
        """
        return true if x (position of the landmark in the map) and y (position of the observed landmark) are compatible
        :param id_x: id of the landmark (estimation)
        :param id_z: id of the landmark (observation)
        :param Z: observation vector
        :return: True if they are compatible, False otherwise
        """
        if id_x == -1:
            return 2 * ((2 * MAX_MATCH_DIST) ** 2)
        else:
            return np.sum(
                np.absolute(Z[id_z:id_z + 2, 0] - self.slamInstance.X[id_x: id_x + 2, 0]))

    def JCBB_wrapper(self, Z):
        """
        associate a sequence of Z measurments to ids in map
        :param Z: measurment made
        :return: list of id_Z and id_X pairs
        """
        idX = self.slamInstance.idx[:]
        idZ = list(range(0, len(Z) - 1, 2))
        (best_H, best_score) = self.JCBB(None, idZ, idX, Z, np.inf, [])
        return best_H

    def JCBB(self, Hyp, ids_Z, ids_X, Z, best_score, best_Hyp):
        """
        find pairing using branch and bound.
        It construct the solution H, by matching elements from Z with elements from X.
        :param Hyp: the current pairings (mat format: 2 columns first for id_Z, second for id_X)
        :param ids_Z: ids of the observation to be matched
        :param ids_X: ids in the map that can still be matched
        :param best_score: the best score
        :return: a tuple containing the best score an the best pairing
        """
        if ids_X is not None:
            ids_X = ids_X[:]
        else:
            ids_X = []
        # leaf node exploration
        if len(ids_Z) == 0:
            current_score = self.joint_compat_score(Hyp, Z)
            if current_score < best_score:
                return Hyp, current_score
            else:
                return best_Hyp, best_score
        else:
            # depth first exploration
            # sort by individual distance
            id_z = ids_Z.pop()
            # # crappy fix
            # if (idX is None) or (idX is []):
            #     return best_Hyp, best_score
            pot_match_X = ids_X[:]
            pot_match_X = sorted(pot_match_X, key=lambda id_x: self.individual_compatibility_score(id_x, id_z, Z))
            pot_match_X = filter(
                lambda id_x: self.individual_compatibility_score(id_x, id_z, Z) < 2 * (MAX_MATCH_DIST ** 2),
                pot_match_X)
            if len(pot_match_X) == 0:
                pot_match_X = [-1]  # case where no x is associated to z0
            for id_x in pot_match_X:
                # note: this condition may be too heavy to evaluate !
                if Hyp is None:
                    next_H = np.mat([[id_x, id_z]])
                else:
                    next_H = np.matrix(Hyp, copy=True)
                    next_H = np.vstack((next_H, np.mat([[id_x, id_z]])))
                if self.joint_compat_score(
                        next_H, Z) < best_score:  # assuming the joint compat grows monotonically with the depth
                    # print("H:", H)
                    # print("next H", next_H)
                    # print("pot x", pot_match_X)
                    # print("x:", x)
                    if id_x != -1:
                        next_X = ids_X.remove(id_x)  # as x is matched, we won't be able to match it after
                    else:
                        next_X = ids_X
                    (new_H, new_best) = self.JCBB(next_H, ids_Z, next_X, Z, best_score, best_Hyp)
                    if new_best < best_score:
                        best_Hyp = new_H
                        best_score = new_best
            return best_Hyp, best_score
