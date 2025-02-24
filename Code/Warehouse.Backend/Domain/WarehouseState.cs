using Models;

namespace Warehouse.Backend.Domain
{
    public class WarehouseState
    {
        public List<Order> OrderList { get; }

        public IReadOnlyList<GoodsItem> GoodsList { get; }

        public WarehouseState()
        {
            OrderList = [];

            GoodsList = new List<GoodsItem>
            {
                new GoodsItem("Прокладка", "img/prokladki.png"),
                new GoodsItem("Болт", "img/bolt.png"),
                new GoodsItem("Гайка", "img/gayki.png"),
                new GoodsItem("Брусок", "img/brusok.png"),
                new GoodsItem("Шайба", "img/shayba.png")
            };
        }
    }
}
