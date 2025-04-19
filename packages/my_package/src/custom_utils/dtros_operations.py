import rospy

class DtrosOperations:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(DtrosOperations, cls).__new__(cls)
        return cls._instance

    @staticmethod
    def unregister_and_delete_subscribers(dtros):
        for attr_name in dir(dtros):
            if attr_name.startswith("_sub_"):
                attr = getattr(dtros, attr_name)
                if hasattr(attr, "unregister") and callable(attr.unregister):
                    try:
                        attr.unregister()
                        print(f"Unregistered {attr_name}")
                        delattr(dtros, attr_name)
                        print(f"Deleted {attr_name}")
                    except Exception as e:
                        rospy.logwarn(f"Failed to unregister {attr_name}: {e}")