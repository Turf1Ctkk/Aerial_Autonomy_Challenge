/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)
*/
#pragma once

#include <Eigen/Eigen>
#include <vector>

class Piece
{
public:
    typedef Eigen::MatrixXd CoefficientMat;

private:
    double duration;
    CoefficientMat coeffMat;
    int DIM;
    int DEGREE;

public:
    Piece() = default;

    Piece(double dur, const CoefficientMat &cMat)
        : duration(dur), DIM(cMat.rows()), DEGREE(cMat.cols() - 1)
    {
        coeffMat = cMat;
    }

    inline int getDim() const { return DIM; }
    inline int getDegree() const { return DEGREE; }
    inline double getDuration() const { return duration; }
    inline const CoefficientMat &getCoeffMat() const { return coeffMat; }

    inline Eigen::VectorXd getPos(const double &t) const
    {
        Eigen::VectorXd pos(DIM);
        pos.setZero();
        double tn = 1.0;
        for (int i = DEGREE; i >= 0; i--)
        {
            pos += tn * coeffMat.col(i);
            tn *= t;
        }
        return pos;
    }

    inline Eigen::VectorXd getVel(const double &t) const
    {
        Eigen::VectorXd vel(DIM);
        vel.setZero();
        double tn = 1.0;
        int n = 1;
        for (int i = DEGREE - 1; i >= 0; i--)
        {
            vel += n * tn * coeffMat.col(i);
            tn *= t;
            n++;
        }
        return vel;
    }

    inline Eigen::VectorXd getAcc(const double &t) const
    {
        Eigen::VectorXd acc(DIM);
        acc.setZero();
        double tn = 1.0;
        int m = 1;
        int n = 2;
        for (int i = DEGREE - 2; i >= 0; --i)
        {
            acc += m * n * tn * coeffMat.col(i);
            tn *= t;
            ++m;
            ++n;
        }
        return acc;
    }

    inline Eigen::VectorXd getJer(const double &t) const
    {
        Eigen::VectorXd jer(DIM);
        jer.setZero();
        double tn = 1.0;
        int l = 1;
        int m = 2;
        int n = 3;
        for (int i = DEGREE - 3; i >= 0; --i)
        {
            jer += l * m * n * tn * coeffMat.col(i);
            tn *= t;
            ++l;
            ++m;
            ++n;
        }
        return jer;
    }

    inline Eigen::VectorXd getSnap(const double &t) const
    {
        Eigen::VectorXd snap(DIM);
        snap.setZero();
        double tn = 1.0;
        int l = 1;
        int m = 2;
        int n = 3;
        int o = 4;
        for (int i = DEGREE - 4; i >= 0; --i)
        {
            snap += o * l * m * n * tn * coeffMat.col(i);
            tn *= t;
            ++l;
            ++m;
            ++n;
            ++o;
        }
        return snap;
    }

    inline Eigen::VectorXd getCrackle(const double &t) const
    {
        Eigen::VectorXd crackle(DIM);
        crackle.setZero();
        double tn = 1.0;
        int l = 1;
        int m = 2;
        int n = 3;
        int o = 4;
        int p = 5;
        for (int i = DEGREE - 5; i >= 0; --i)
        {
            crackle += p * o * l * m * n * tn * coeffMat.col(i);
            tn *= t;
            ++l;
            ++m;
            ++n;
            ++o;
            ++p;
        }
        return crackle;
    }
};

class Trajectory
{
private:
    typedef std::vector<Piece> Pieces;
    Pieces pieces;

public:
    Trajectory() = default;

    Trajectory(const std::vector<double> &durs,
               const std::vector<Piece::CoefficientMat> &cMats)
    {
        int N = std::min(durs.size(), cMats.size());
        pieces.reserve(N);
        for (int i = 0; i < N; i++)
        {
            pieces.emplace_back(durs[i], cMats[i]);
        }
    }

    inline int getPieceNum() const { return pieces.size(); }
    inline int getDim() const { return pieces[0].getDim(); }

    inline double getTotalDuration() const
    {
        int N = getPieceNum();
        double totalDuration = 0.0;
        for (int i = 0; i < N; i++)
        {
            totalDuration += pieces[i].getDuration();
        }
        return totalDuration;
    }

    inline const Piece &operator[](int i) const { return pieces[i]; }
    inline Piece &operator[](int i) { return pieces[i]; }

    inline void clear(void) { pieces.clear(); }
    inline Pieces::const_iterator begin() const { return pieces.begin(); }
    inline Pieces::const_iterator end() const { return pieces.end(); }
    inline Pieces::iterator begin() { return pieces.begin(); }
    inline Pieces::iterator end() { return pieces.end(); }
    inline void reserve(const int &n) { pieces.reserve(n); }
    inline void emplace_back(const Piece &piece) { pieces.emplace_back(piece); }
    inline void emplace_back(const double &dur, const Piece::CoefficientMat &cMat) { pieces.emplace_back(dur, cMat); }

    inline int locatePieceIdx(double &t) const
    {
        int N = getPieceNum();
        int idx;
        double dur;
        for (idx = 0; idx < N && t > (dur = pieces[idx].getDuration()); idx++)
        {
            t -= dur;
        }
        if (idx == N)
        {
            idx--;
            t += pieces[idx].getDuration();
        }
        return idx;
    }

    inline Eigen::VectorXd getPos(double t) const
    {
        int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getPos(t);
    }

    inline Eigen::VectorXd getVel(double t) const
    {
        int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getVel(t);
    }

    inline Eigen::VectorXd getAcc(double t) const
    {
        int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getAcc(t);
    }

    inline Eigen::VectorXd getJer(double t) const
    {
        int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getJer(t);
    }

    inline Eigen::MatrixXd getPVAJSC(double t) const
    {
        int pieceIdx = locatePieceIdx(t);
        Eigen::MatrixXd pvajsc(getDim(), 6);
        pvajsc.col(0) = pieces[pieceIdx].getPos(t);
        pvajsc.col(1) = pieces[pieceIdx].getVel(t);
        pvajsc.col(2) = pieces[pieceIdx].getAcc(t);
        pvajsc.col(3) = pieces[pieceIdx].getJer(t);
        pvajsc.col(4) = pieces[pieceIdx].getSnap(t);
        pvajsc.col(5) = pieces[pieceIdx].getCrackle(t);
        return pvajsc;
    }

    inline Eigen::VectorXd getJuncPos(int juncIdx) const
    {
        if (juncIdx != getPieceNum())
        {
            return pieces[juncIdx].getCoeffMat().col(pieces[juncIdx].getDegree());
        }
        else
        {
            return pieces[juncIdx - 1].getPos(pieces[juncIdx - 1].getDuration());
        }
    }
};
