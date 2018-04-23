import numpy as np

MAX_MULTI_DIST = 0.4
JOINT_PENALITY = 100
MAX_INDIV_DIST = 4

MAX_RANGE = 10
MIN_RANGE = 0


class DataAssociator:

    def __init__(self, laserSLAM):
        self.slamInstance = laserSLAM

    @staticmethod
    def joint_compat_score(hypothesis, Z, X, d_z, d_x):
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
                # print("row:"+str(row))
                id_x = row[0, 0]
                id_z = row[0, 1]
                # Z_expected = X[id_x: id_x + 2, 0]
                # Z_observed = Z[id_z : id_z + 2, 0]
                res += DataAssociator.multi_compat_score(hypothesis, id_x, id_z, Z, X, d_z, d_x)
                # res += np.sum(np.square(Z_expected - Z_observed))
            else:
                res += JOINT_PENALITY
        return res  # sum([(lx_x - lz_x) ** 2 + (lx_y - lz_y) ** 2 for ((lx_x, lx_y), (lz_x, lz_y)) in pairings])

    @staticmethod
    def individual_compatibility_score(id_x, id_z, Z, X):
        """
        return true if x (position of the landmark in the map) and y (position of the observed landmark) are compatible
        :param id_x: id of the landmark (estimation)
        :param id_z: id of the landmark (observation)
        :param Z: observation vector
        :return: True if they are compatible, False otherwise
        """
        if id_x == -1:
            return 2 * ((2 * MAX_INDIV_DIST) ** 2)
        else:
            return np.sum(
                np.absolute(Z[id_z:id_z + 2, 0] - X[id_x: id_x + 2, 0])
            )

    @staticmethod
    def multi_compat_score(H, id_x, id_z, Z, X, d_z, d_x):
        """
        check if the inter-distance constraint is verified
        :param H:
        :param id_x:
        :param id_z:
        :param Z:
        :return:
        """
        error = 0
        if (H is None) or (id_x == -1):
            return np.inf
        for row in H:
            id_xi = row[0, 0]
            id_zi = row[0, 1]
            if id_xi != -1:
                error += d_x[min(id_x, id_xi), max(id_x, id_xi)] - \
                         d_z[min(id_z, id_zi), max(id_z, id_zi)] \
                         ** 2
            # else:
                # error += MAX_MATCH_DIST
        return error / len(H)

    def JCBB_wrapper(self, Z):
        """
        associate a sequence of Z measurments to ids in map
        :param Z: measurment made
        :return: list of id_Z and id_X pairs
        """
        idX = []
        idZ = list(range(0, len(Z) - 1, 2))
        X = np.mat(np.zeros_like(self.slamInstance.X))
        h = self.slamInstance.get_h()
        for idx in self.slamInstance.idx:
            landmark = h(self.slamInstance.X[idx:idx+2, 0])
            X[idx:idx+2, 0] = landmark
            if (abs(landmark[0, 0]) < MAX_RANGE) and (abs(landmark[1, 0]) < MAX_RANGE)\
                    and (abs(landmark[0, 0]) > MIN_RANGE) and (abs(landmark[1, 0]) > MIN_RANGE):
                idX.append(idx)
        d_x = {}
        for i in range(0, len(idX), 1):
            for j in range(i, len(idX), 1):
                diff = (X[idX[i]:idX[i]+2, 0] - X[idX[j]:idX[j]+2, 0])
                d_x[(idX[i], idX[j])] = np.sqrt(np.sum(np.square(diff)))
        d_z = {}
        for i in range(0, len(idZ), 1):
            for j in range(i, len(idZ), 1):
                diff = (Z[idZ[i]:idZ[i]+2, 0] - Z[idZ[j]:idZ[j]+2, 0])
                d_z[(idZ[i], idZ[j])] = np.sqrt(np.sum(np.square(diff)))
        # print("***************************************************")
        # print("trace:")
        # print("ids_Z:")
        # print(idZ)
        # print("ids_X:")
        # print(idX)
        # print("Z:")
        # print(Z)
        # print("***************************************************")
        (best_H, best_score) = self.JCBB(None, idZ, idX, Z, X, d_z, d_x, np.inf, [])
        return best_H

    def JCBB(self, Hyp, ids_Z, ids_X, Z, X, d_z, d_x, best_score, best_Hyp):
        """
        find pairing using branch and bound.
        It construct the solution H, by matching elements from Z with elements from X.
        :param Hyp: the current pairings (mat format: 2 columns first for id_Z, second for id_X)
        :param ids_Z: ids of the observation to be matched
        :param ids_X: ids in the map that can still be matched
        :param best_score: the best score
        :return: a tuple containing the best score an the best pairing
        """
        if ids_X is None:
            ids_X = []
        # leaf node exploration
        if len(ids_Z) == 0:
            current_score = self.joint_compat_score(Hyp, Z, X, d_z, d_x)
            if current_score < best_score:
                return Hyp, current_score
            else:
                return best_Hyp, best_score
        else:
            # depth first exploration
            # sort by individual distance
            ids_Z = ids_Z[:]
            id_z = ids_Z.pop()
            # # crappy fix
            # if (idX is None) or (idX is []):
            #     return best_Hyp, best_score
            pot_match_X = ids_X[:]
            # pot_match_X = sorted(pot_match_X, key=lambda id_x: self.multi_compat_score(Hyp, id_x, id_z, Z, X, d_z, d_x))
            pot_match_X = filter(
                lambda id_x: self.individual_compatibility_score(id_x, id_z, Z, X) < MAX_INDIV_DIST,
                pot_match_X)
            if Hyp is not None:
                pot_match_X = filter(
                    lambda id_x:
                    all(
                        True if row[0, 0] == -1 else (abs(d_z[min(id_z, row[0, 1]), max(id_z, row[0, 1])] - d_x[min(id_x, row[0, 0]), max(id_x, row[0, 0])]) < MAX_MULTI_DIST)
                        for row in Hyp
                    ),
                       pot_match_X)
            # pot_match_X = filter(
            #     lambda id_x: self.multi_compat_score(Hyp, id_x, id_z, Z, X, d_z, d_x) < MAX_MULTI_DIST ** 2,
            #     pot_match_X)
            if len(pot_match_X) == 0:
                pot_match_X = [-1]  # case where no x is associated to z0
            for id_x in pot_match_X:
                if Hyp is None:
                    next_H = np.mat([[id_x, id_z]])
                else:
                    next_H = np.matrix(Hyp, copy=True)
                    next_H = np.vstack((next_H, np.mat([[id_x, id_z]])))
                if self.joint_compat_score(
                        next_H, Z, X, d_z, d_x) <= best_score:  # assuming the joint compat grows monotonically with the depth
                    # print("H:", H)
                    # print("next H", next_H)
                    # print("pot x", pot_match_X)
                    # print("x:", x)
                    if id_x != -1:
                        next_X = ids_X[:]
                        next_X.remove(id_x)  # as x is matched, we won't be able to match it after
                    else:
                        next_X = ids_X[:]
                    (new_H, new_best) = self.JCBB(next_H, ids_Z, next_X, Z, X, d_z, d_x, best_score, best_Hyp)
                    if new_best < best_score:
                        best_Hyp = new_H
                        best_score = new_best
            return best_Hyp, best_score
