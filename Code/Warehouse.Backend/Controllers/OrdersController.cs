using Microsoft.AspNetCore.Mvc;
using Warehouse.Backend.Domain;
using Models;


namespace Warehouse.Backend.Controllers
{
    [ApiController]
    [Route("[controller]")]
    public class OrdersController(WarehouseState warehouseState) : ControllerBase
    {
        private readonly WarehouseState _warehouseState = warehouseState;
        private int counter = 0;

        [Route("get")]
        [HttpGet]
        public ActionResult<Order> GetLastOrder()
        {
            if (_warehouseState.OrderList.Count == 0)
            {
                return Ok("This site is working");
            }

            return _warehouseState.OrderList.Last();
        }

        [Route("get/app")]
        [HttpGet]
        public ActionResult<Order> GetLastOrderForApp()
        {
            if (_warehouseState.OrderList.Last().Status != "LocationInfo")
            {
                return BadRequest(new { message = "Ошибка запроса", code = 400 });
            }

            return _warehouseState.OrderList.Last();
        }

        [Route("get/robot")]
        [HttpGet]
        public ActionResult<Order> GetLastOrderForRobot()
        {
            if (_warehouseState.OrderList.Last().Status != "InProgress")
            {
                return BadRequest(new { message = "Ошибка запроса", code = 400 });
            }

            var _lastOrder = _warehouseState.OrderList.Last();
            _lastOrder.Status = "OnRobot";

            return _lastOrder;
        }

        [Route("create")]
        [HttpPost]
        public ActionResult<Order> CreateOrder()
        {
            var items = _warehouseState.GoodsList
                .Select((goods, index) => new OrderItem(goods, 0))
                .ToList();

            var newOrder = new Order(items);
            _warehouseState.OrderList.Add(newOrder);

            return newOrder;
        }

        [Route("update/count")]
        [HttpPut]
        public ActionResult<Order> UpdateLastOrder([FromBody] int[] nums)
        {
            if (nums.Length != 5)
            {
                return BadRequest("Массив должен содержать ровно 5 чисел.");
            }

            if (_warehouseState.OrderList.Count == 0)
            {
                return NotFound("Заказов нет.");
            }

            var lastOrder = _warehouseState.OrderList.Last();

            for (var i = 0; i < 5; i++)
            {
                lastOrder.Items[i].Quantity = nums[i];
            }

            lastOrder.Status = "InProgress";

            return lastOrder;
        }

        [Route("update/loc")]
        [HttpPut]
        public ActionResult<Order> UpdateItemsLocation([FromBody] UpdateLocationRequest request)
        {
            if (request.Locations.Length != 3)
            {
                return BadRequest("Массив должен содержать ровно 3 числа.");
            }

            if (_warehouseState.OrderList.Count == 0)
            {
                return NotFound("Заказов нет.");
            }

            var lastOrder = _warehouseState.OrderList.Last();
            var itemToUpdate = lastOrder.Items.FirstOrDefault(item => item.Goods.Name == request.GoodsName);

            if (itemToUpdate == null)
            {
                return NotFound("Товар с таким именем не найден в последнем заказе.");
            }

            itemToUpdate.Rack = request.Locations[0];
            itemToUpdate.Cell = request.Locations[1];
            itemToUpdate.Shelf = request.Locations[2];

            if (counter > 3)
            {
                counter = 0;
                lastOrder.Status = "LocationInfo";
            }

            counter++;
            return lastOrder;
        }

        [Route("put/done")]
        [HttpPut]
        public void AllDone()
        {
            _warehouseState.OrderList.Last().Status = "Finish";
        }
    }
}
