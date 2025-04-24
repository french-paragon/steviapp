#include <QtTest/QtTest>

#include "vision/indexed_timed_sequence.h"

class TestIndexedTimeSequence: public QObject
{
    Q_OBJECT
private Q_SLOTS:
    void testGetValueAtTime();
};

void TestIndexedTimeSequence::testGetValueAtTime() {

    using TimedElement = StereoVisionApp::IndexedTimeSequence<double>::TimedElement;

    constexpr int nElements = 10;

    std::vector<TimedElement> data(nElements);

    for (int i = 0; i < nElements; i++) {
        data[i].time = i;
        data[i].val = i;
    }

    StereoVisionApp::IndexedTimeSequence<double> sequence(data);

    //check elements within the sequence
    for (int i = 0; i < nElements-1; i++) {

        std::array<double,5> dts = {0, 0.25, 0.5, 0.75};

        for (double dt : dts) {

            double time = i + dt;

            auto interpolable = sequence.getValueAtTime(time);

            double val = interpolable.weigthLower*interpolable.valLower + interpolable.weigthUpper*interpolable.valUpper;

            QCOMPARE(val, time);

        }

    }

    //check elements outside of the sequence (nearest interpolation is applied outside the sequence)
    std::array<double,5> dts = {0.25, 0.5, 0.75};

    for (double dt : dts) {

        double time0 = -dt;
        double timeN = nElements-1 + dt;

        auto interpolable0 = sequence.getValueAtTime(time0);
        auto interpolableN= sequence.getValueAtTime(timeN);

        double val0 = interpolable0.weigthLower*interpolable0.valLower + interpolable0.weigthUpper*interpolable0.valUpper;
        double valN = interpolableN.weigthLower*interpolableN.valLower + interpolableN.weigthUpper*interpolableN.valUpper;

        QCOMPARE(val0, 0);
        QCOMPARE(valN, nElements-1);
    }

}

QTEST_MAIN(TestIndexedTimeSequence)
#include "testIndexedTimeSequence.moc"
