import pytest
import image_saver


class TestImageSaver(object):
    @pytest.fixture
    def image_queue(self):
        def gen_image_queue(max=10):
            iq = image_saver.FixSizeDefaultStrDict(dict, max_len=max)

            for el in range(10, 20):
                for cam in range(3):
                    iq[str(el)][cam] = el - cam
            return iq

        return gen_image_queue

    def testFixSizeDefaultStrDict(self):
        f = image_saver.FixSizeDefaultStrDict(dict, max=3)

        for i in range(5):
            for j in range(5):
                f[str(i)][str(j)] = "{},{}".format(i, j)

        assert len(f) == 3

    def test_simple_get_oldest_unprocessed(self, image_queue):
        iq = image_queue(3)
        for ts in range(10):
            for cam in range(3):
                iq[str(ts)][cam] = "{}, {}".format(ts, cam)

        assert image_saver.get_oldest_unprocessed(iq) == str(17)

    def test_starting_get_oldest_unprocessed(self, image_queue):
        # Check for a newly started image queue
        iq = image_queue()
        del iq["10"][2]
        del iq["10"][1]
        del iq["11"][2]
        assert image_saver.get_oldest_unprocessed(iq) == str(10)
        iq = image_saver.FixSizeDefaultStrDict(dict, max_len=10)
        iq["10"][0] = 10
        assert image_saver.get_oldest_unprocessed(iq) is None
        iq["11"] = {1: 10, 0: 11}
        assert image_saver.get_oldest_unprocessed(iq) is None

    def test_broken_get_oldest_unprocessed(self, image_queue):
        """ Check that the indexes make sense even if there are holes in the
        image queue.
        """

        # Check for a full image queue
        iq = image_queue()
        assert image_saver.get_oldest_unprocessed(iq) == str(10)
        del iq["12"][0]
        assert image_saver.get_oldest_unprocessed(iq) == str(10)
        del iq["11"][1]
        assert image_saver.get_oldest_unprocessed(iq) == str(11)
        del iq["13"][2]
        assert image_saver.get_oldest_unprocessed(iq) == str(13)
        for i in range(10, 18):
            del iq[str(i)]
        assert image_saver.get_oldest_unprocessed(iq) is None
        iq = image_saver.FixSizeDefaultStrDict(dict, max_len=10)
        assert image_saver.get_oldest_unprocessed(iq) is None

    def test_extract_image_chain(self, image_queue):
        iq = image_queue()
        chain = image_saver.extract_image_chain(iq, "10")
        assert "".join(map(str, chain)) == "101010"

    def test_starting_extract_image_chain(self, image_queue):
        iq = image_queue()
        del iq["10"][2]
        del iq["10"][1]
        del iq["11"][2]
        chain = image_saver.extract_image_chain(iq, "10")
        assert chain == [10, 10, 10]
        iq = image_saver.FixSizeDefaultStrDict(dict, max_len=10)
        iq["10"][0] = 10
        assert image_saver.extract_image_chain(iq, "10") is None
        iq["11"] = {1: 10, 0: 11}
        assert image_saver.extract_image_chain(iq, "10") is None
        iq["12"] = {2: 10, 1: 11, 0: 12}
        ts = image_saver.get_oldest_unprocessed(iq)
        assert image_saver.extract_image_chain(iq, ts) == [10, 10, 10]


    def test_broken_extract_image_chain(self, image_queue):
        """ Check that the indexes make sense even if there are holes in the
        image queue.
        """

        # Check for a full image queue
        iq = image_queue()
        ts = image_saver.get_oldest_unprocessed(iq)
        assert image_saver.extract_image_chain(iq, ts) == [10, 10, 10]
        del iq["12"][0]
        ts = image_saver.get_oldest_unprocessed(iq)
        assert image_saver.extract_image_chain(iq, ts) == [10, 10, 10]
        del iq["11"][1]
        ts = image_saver.get_oldest_unprocessed(iq)
        assert image_saver.extract_image_chain(iq, ts) == [11, 11, 11]
        del iq["13"][2]
        ts = image_saver.get_oldest_unprocessed(iq)
        assert image_saver.extract_image_chain(iq, ts) == [13, 13, 13]
        for i in range(10, 18):
            del iq[str(i)]
        print(iq)
        assert image_saver.extract_image_chain(iq, "18") is None
        ts = image_saver.get_oldest_unprocessed(iq)
        with pytest.raises(ValueError):
            assert image_saver.extract_image_chain(iq, ts) is None
        iq = image_saver.FixSizeDefaultStrDict(dict, max_len=10)
        ts = image_saver.get_oldest_unprocessed(iq)
        with pytest.raises(ValueError):
            assert image_saver.extract_image_chain(iq, ts) is None
