#ifndef INDEXED_TIMED_SEQUENCE_H
#define INDEXED_TIMED_SEQUENCE_H

#include <vector>
#include <algorithm>
#include <cmath>

namespace StereoVisionApp {

/*!
 * \brief The IndexedTimeSequence class contain an indexed time sequence of elements T
 *
 * The class is meant to quickly access values in a time sequence.
 */
template<typename T, typename TimeT = double>
class IndexedTimeSequence {

public:

    struct TimedElement {
        TimeT time;
        T val;
    };

    /*!
     * \brief The TimeInterpolableVals struct contain a pair of values for interpolation
     *
     * The IndexedTimeSequence do not care about interpolation directly, as the end user might want to use specific approaches.
     * For example, some values might have to be interpolated on a manifold. So when asking for a value at a specific time,
     * the IndexedTimeSequence will return a pair of values with weights that the user can combine anyway they like.
     */
    struct TimeInterpolableVals {
        double weigthLower;
        T valLower;
        double weigthUpper;
        T valUpper;
    };

    struct Differential {
        double dt;
        T val;
    };

    /*!
     * \brief IndexedTimeSequence construct an empty sequence.
     */
    IndexedTimeSequence() :
        _initialTime(1),
        _finalTime(0),
        _indicesFromTimeIntervals(),
        _sequence()
    {

    }

    /*!
     * \brief IndexedTimeSequence copy constructor from vector
     * \param sequence
     */
    IndexedTimeSequence(std::vector<TimedElement> const& sequence) :
        _sequence(sequence)
    {
        rebuildIndex();
    }

    /*!
     * \brief IndexedTimeSequence move constructor from vector
     * \param sequence the sequence.
     */
    IndexedTimeSequence(std::vector<TimedElement> && sequence) :
        _sequence(sequence)
    {
        rebuildIndex();
    }

    /*!
     * \brief IndexedTimeSequence copy constructor from IndexedTimeSequence
     * \param other
     */
    IndexedTimeSequence(IndexedTimeSequence<T> const& other) :
        _initialTime(other._initialTime),
        _finalTime(other._finalTime),
        _indicesFromTimeIntervals(other._indicesFromTimeIntervals),
        _sequence(other._sequence)
    {

    }

    /*!
     * \brief IndexedTimeSequence move constructor from vector
     * \param sequence the sequence.
     */
    IndexedTimeSequence(IndexedTimeSequence<T> && other) :
        _initialTime(other._initialTime),
        _finalTime(other._finalTime),
        _indicesFromTimeIntervals(std::move(other._indicesFromTimeIntervals)),
        _sequence(std::move(other._sequence))
    {

    }

    void rebuildIndex(int nElements = -1) {
        int nElems = nElements;

        if (nElems <= 0) {
            nElems = _sequence.size();
        }

        std::sort(_sequence.begin(), _sequence.end(), [] (TimedElement const& t1, TimedElement const& t2){
            return t1.time < t2.time;
        });

        _initialTime = _sequence.front().time;
        _finalTime = _sequence.back().time;

        _indicesFromTimeIntervals.resize(nElems);
        TimeT deltaT = indexTimePerStep();

        int p = 0;

        for (int i = 0; i < nElems; i++) {

            while (_sequence[p].time < _initialTime+i*deltaT) {
                p++;
            }

            _indicesFromTimeIntervals[i] = p;
        }
    }

    TimeT indexTimePerStep() const {
        return (_finalTime - _initialTime)/_indicesFromTimeIntervals.size();
    }

    TimeInterpolableVals getValueAtTime(TimeT time) const {

        if (time <= _initialTime) {
            return TimeInterpolableVals{0, T(), 1, _sequence.front().val};
        }

        if (time >= _finalTime) {
            return TimeInterpolableVals{1, _sequence.back().val, 0, T()};
        }

        double deltaT = indexTimePerStep();
        double localTime = time - _initialTime;

        int initialIndexIdx = static_cast<int>(std::floor(localTime/deltaT));
        int initialSearchIdx = _indicesFromTimeIntervals[initialIndexIdx];

        initialSearchIdx = std::max(0, initialSearchIdx-1);

        for (int i = initialSearchIdx; i < _sequence.size()-1; i++) {

            if (_sequence[i].time <= time and _sequence[i+1].time >= time) {
                double timeDelta = _sequence[i+1].time - _sequence[i].time;
                double preDelta = time - _sequence[i].time;
                double postDelta = _sequence[i+1].time - time;

                double wPre = preDelta/timeDelta;
                double wPost = postDelta/timeDelta;

                if (!std::isfinite(wPre) and !std::isfinite(wPost)) {
                    wPre = 0.5;
                    wPost = 0.5;
                }

                return {wPre, _sequence[i].val, wPost, _sequence[i+1].val};
            }

        }

        return TimeInterpolableVals{1, _sequence.back().val, 0, T()};
    }

    std::vector<Differential> getValuesInBetweenTimes(TimeT t0, TimeT tf) const {

        if (t0 >= _finalTime) {
            return std::vector<Differential>{Differential{tf - t0, _sequence.back().val}};
        }

        double deltaT = indexTimePerStep();
        double localT0 = t0 - _initialTime;

        int initialIndexIdx = static_cast<int>(std::floor(localT0/deltaT));

        if (initialIndexIdx < 0) {
            initialIndexIdx = 0;
        }

        int initialSearchIdx = _indicesFromTimeIntervals[initialIndexIdx];

        initialSearchIdx = std::max(0, initialSearchIdx-1);

        int initialIndex = initialSearchIdx;

        while (initialIndex < _sequence.size() and _sequence[initialIndex].time < t0) {
            initialIndex++;
        }

        int finalIndex = initialIndex;

        while (finalIndex < _sequence.size() and _sequence[finalIndex].time < tf) {
            finalIndex++;
        }

        if (initialIndex > 0) {
            initialIndex--;
            finalIndex--;
        }

        //now all the elements we need to return as differentials are within initialIndex and finalIndex
        if (finalIndex == initialIndex) {
            return std::vector<Differential>{Differential{tf - t0, _sequence[initialIndex].val}};
        }

        int nElements = finalIndex - initialIndex + 1;
        double t = t0;

        int currentIndex = initialIndex;

        std::vector<Differential> ret(nElements);

        for (int i = 0; i < nElements; i++) {

            if (currentIndex+1 > _sequence.size()) {
                ret[i] = Differential{tf - t, _sequence[currentIndex].val};
                break;
            }

            if (_sequence[currentIndex+1].time > tf) {
                ret[i] = Differential{tf - t, _sequence[currentIndex].val};
                break;
            }

            ret[i] = Differential{_sequence[currentIndex+1].time - t, _sequence[currentIndex].val};

            t = _sequence[currentIndex+1].time;
            currentIndex++;
        }

        return ret;

    }

    TimeT sequenceStartTime() const {
        return _initialTime;
    }

    TimeT sequenceEndTime() const {
        return _finalTime;
    }

    int nPoints() const {
        return _sequence.size();
    }

    TimedElement operator[](int idx) {
        return _sequence[idx];
    }

    bool isValid() const {
        return _initialTime <= _finalTime;
    }

protected:

    TimeT _initialTime;
    TimeT _finalTime;

    std::vector<int> _indicesFromTimeIntervals;
    std::vector<TimedElement> _sequence;

};

}

#endif // INDEXED_TIMED_SEQUENCE_H
