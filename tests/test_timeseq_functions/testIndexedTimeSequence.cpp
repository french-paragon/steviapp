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

    for (int i = 0; i < nElements-1; i++) {

        std::array<double,3> dts = {0.25, 0.5, 0.75};

        for (double dt : dts) {

            double time = i + dt;

            auto interpolable = sequence.getValueAtTime(time);

            double val = interpolable.weigthLower*interpolable.valLower + interpolable.weigthUpper*interpolable.valUpper;

            QCOMPARE(val, time);

        }

    }

}

QTEST_MAIN(TestIndexedTimeSequence)
#include "testIndexedTimeSequence.moc"
